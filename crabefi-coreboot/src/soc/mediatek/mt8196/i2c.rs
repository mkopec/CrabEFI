//! MT8196 I2C buses and the board I2C peripherals: TPM, touchpad.
//!
//! The controllers are crabefi-hw's MT65xx driver; this module clocks them,
//! sets pads and power, and gives each a bounce page from the
//! non-cacheable DMA pool. The Linux mt8188/mt8196 I2C has no 255-byte
//! message quirk, so transfers up to a page are allowed.
//!
//! - I2C1 (Ti50 TPM @0x50): pads and 1 MHz timing set by coreboot's bootblock,
//!   used as left; EINT18 (rising edge, configured by coreboot) is polled as
//!   the TPM's ready latch.
//! - I2C0 (touchpad, FocalTech HID @0x38 or Elan @0x15): untouched by
//!   coreboot. CrabEFI ungates its clocks, configures pads 99/100 as Linux's
//!   `i2c0_pins` (they reset with pull-downs), programs the controller like
//!   Linux's i2c-mt65xx with 400 kHz timing from coreboot's algorithm, and
//!   powers the touchpad: MT6373 VFP (`pp1800_vfp`, the 1.8 V side; without
//!   it the touchpad ACKs but returns zeros) and VCN33_2 (`pp3300_tchpad`).
//!   GPIO8 (level low) is the touchpad interrupt.

use alloc::{boxed::Box, rc::Rc, string::String, vec, vec::Vec};
use core::cell::RefCell;
use crabefi::peripheral::{
    I2cBus, I2cPeripheral, InputLine, Interrupt, InterruptLatch, Peripheral,
};
use crabefi::time::{Delay, Timeout, delay_us};
use crabefi_hw::mediatek::i2c::{MtkI2c, Timing};

use super::gpio;
use embedded_hal::digital;
use embedded_hal_bus::i2c::RcDevice;

const EINT_S: u64 = 0x1288_0000;
const CKSYS: u64 = 0x1000_0000;
const IMP_IIC_WRAP_W: u64 = 0x131b_0000;
/// coreboot PMIF_SPMI_AP_CHAN (SPMI master 1, as coreboot's mt6373 driver).
const PMIF_CHAN: u64 = 0x1c01_a000 + 0x880;
const MT6373_SID: u32 = 5;

/// I2C source clock (coreboot mt8196 I2C_CLK_HZ).
const I2C_CLK_HZ: u32 = 124_800_000;

type Controller = MtkI2c<Delay>;

fn r32(address: u64) -> u32 {
    // SAFETY: Device-mapped MT8196 registers (clocks, IOCFG pads, EINT, PMIF).
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

fn w32(address: u64, value: u32) {
    // SAFETY: as above.
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

// ---------------------------------------------------------------------------
// GPIO / EINT
// ---------------------------------------------------------------------------

/// Plain GPIO input.
struct GpioIn(u32);

impl digital::ErrorType for GpioIn {
    type Error = digital::ErrorKind;
}

impl digital::InputPin for GpioIn {
    fn is_high(&mut self) -> Result<bool, digital::ErrorKind> {
        Ok(gpio::input(self.0))
    }
    fn is_low(&mut self) -> Result<bool, digital::ErrorKind> {
        Ok(!gpio::input(self.0))
    }
}

/// Pad control fields of one pin (Linux pinctrl-mt8196 ranges).
struct Pad {
    pin: u32,
    /// IOCFG block base.
    base: u64,
    /// (offset, bit) of the pull-up and pull-down enables.
    pu: (u64, u32),
    pd: (u64, u32),
    /// (offset, shift) of the 3-bit drive-strength field.
    drv: (u64, u32),
    /// (offset, shift) of the 3-bit advanced-drive field, if any.
    adv: Option<(u64, u32)>,
}

/// Pins the rauru (Navi) devicetrees configure for CrabEFI's devices. Other pins
/// keep whatever coreboot programmed.
const PADS: &[Pad] = &[
    // IOCFG_LB1: touchpad interrupt.
    Pad {
        pin: 8,
        base: 0x130f_0000,
        pu: (0x80, 7),
        pd: (0x70, 7),
        drv: (0x00, 9),
        adv: None,
    },
    // IOCFG_LB2: SCL0/SDA0.
    Pad {
        pin: 99,
        base: 0x1311_0000,
        pu: (0x90, 9),
        pd: (0x80, 9),
        drv: (0x00, 18),
        adv: Some((0x20, 0)),
    },
    Pad {
        pin: 100,
        base: 0x1311_0000,
        pu: (0x90, 12),
        pd: (0x80, 12),
        drv: (0x00, 27),
        adv: Some((0x20, 9)),
    },
];

fn rmw(address: u64, clear: u32, set: u32) {
    w32(address, (r32(address) & !clear) | set);
}

/// Apply devicetree pin groups: pinmux function, `bias-disable` and
/// `drive-strength` (pads reset with pulls enabled, which hold I2C low).
fn configure_pins(groups: &[super::dt::PinGroup]) {
    for group in groups {
        for &pinmux in &group.pinmux {
            let (pin, function) = (pinmux >> 8, pinmux & 0xf);
            gpio::set_mode(pin, function);
            if function == 0 {
                gpio::make_input(pin);
            }
            let pad = PADS.iter().find(|pad| pad.pin == pin);
            let needs_pad = group.bias_disable || group.drive_strength.is_some();
            match pad {
                None if needs_pad => {
                    log::warn!("MT8196 pinctrl: no pad table for GPIO{pin}; bias/drive left as is")
                }
                None => {}
                Some(pad) => {
                    if group.bias_disable {
                        rmw(pad.base + pad.pu.0, 1 << pad.pu.1, 0);
                        rmw(pad.base + pad.pd.0, 1 << pad.pd.1, 0);
                    }
                    if let Some(milliamps) = group.drive_strength {
                        let value = (milliamps / 2).saturating_sub(1).min(7);
                        rmw(pad.base + pad.drv.0, 7 << pad.drv.1, value << pad.drv.1);
                        // As coreboot's gpio_set_driving(): advanced drive off.
                        if let Some((offset, shift)) = pad.adv {
                            rmw(pad.base + offset, 7 << shift, 0);
                        }
                    }
                }
            }
        }
    }
}

/// EINT latch (coreboot gpio_eint_poll): STA at +0x00, ACK at +0x40.
struct EintLatch {
    base: u64,
    bit: u32,
}

impl InterruptLatch for EintLatch {
    fn take(&mut self) -> bool {
        let pending = r32(self.base) & (1 << self.bit) != 0;
        if pending {
            w32(self.base + 0x40, 1 << self.bit);
        }
        pending
    }
}

/// EINT block and index of a GPIO (coreboot mt8196 eint table), for edge
/// interrupts that coreboot configured.
fn eint(gpio: u32) -> Option<EintLatch> {
    match gpio {
        18 => Some(EintLatch {
            base: EINT_S,
            bit: 8,
        }),
        _ => None,
    }
}

/// A devicetree interrupt (`interrupts-extended = <&pio gpio type>`).
fn interrupt(gpio: u32, flags: u32) -> Option<Interrupt> {
    use super::dt::{IRQ_TYPE_EDGE_RISING, IRQ_TYPE_LEVEL_LOW};
    match flags {
        IRQ_TYPE_LEVEL_LOW | 4 => Some(Interrupt::Level {
            line: InputLine::new(GpioIn(gpio)),
            active_low: flags == IRQ_TYPE_LEVEL_LOW,
        }),
        IRQ_TYPE_EDGE_RISING => eint(gpio).map(|latch| Interrupt::Latched(Box::new(latch))),
        _ => None,
    }
    .or_else(|| {
        log::warn!("MT8196: unsupported interrupt GPIO{gpio} type {flags:#x}");
        None
    })
}

/// Make sure a controller is clocked before touching it: registers of a
/// gated block fault. coreboot clocks the north wrap (I2C1, TPM); I2C0 (west
/// wrap) needs the `i2c_west` mux gate (CLK_CFG_11 bit 23, cleared through
/// CLK_CFG_11_CLR) and IMPW_I2C0 (imp_iic_wrap_west CG bit 0, CG_CLR +0xe04),
/// as in Linux clk-mt8196. Other controllers are not used.
fn enable_clocks(base: u64) -> bool {
    match base {
        0x1393_0000 => return true,
        0x1313_0000 => {}
        _ => return false,
    }
    if r32(CKSYS + 0xc0) & (1 << 23) != 0 {
        w32(CKSYS + 0xc8, 1 << 23);
    }
    if r32(IMP_IIC_WRAP_W + 0xe00) & 1 != 0 {
        w32(IMP_IIC_WRAP_W + 0xe04, 1);
    }
    delay_us(10);
    true
}

// ---------------------------------------------------------------------------
// MT6373 over SPMI (coreboot common/pmif.c send_cmd, EXT_REG_LONG)
// ---------------------------------------------------------------------------

fn pmif_wait(state: u32, timeout_us: u64) -> bool {
    let timeout = Timeout::from_us(timeout_us);
    loop {
        if (r32(PMIF_CHAN + 0x28) >> 1) & 7 == state {
            return true;
        }
        if timeout.is_expired() {
            return false;
        }
    }
}

fn spmi_cmd(write: bool, reg: u16, data: u8) -> Option<u8> {
    spmi_cmd_sid(MT6373_SID, write, reg, data)
}

fn spmi_cmd_sid(sid: u32, write: bool, reg: u16, data: u8) -> Option<u8> {
    const IDLE: u32 = 0;
    const WFVLDCLR: u32 = 6;
    if !pmif_wait(IDLE, 1000) {
        log::warn!(
            "MT8196 SPMI: channel not idle (sta={:#x})",
            r32(PMIF_CHAN + 0x28)
        );
        return None;
    }
    let cmd = (3 << 30) | (u32::from(write) << 29) | (sid << 24) | u32::from(reg);
    if write {
        w32(PMIF_CHAN + 0x04, u32::from(data));
    }
    w32(PMIF_CHAN, cmd);
    if write {
        return pmif_wait(IDLE, 1000).then_some(data);
    }
    if !pmif_wait(WFVLDCLR, 1000) {
        log::warn!("MT8196 SPMI: read {reg:#x} timed out");
        return None;
    }
    let value = r32(PMIF_CHAN + 0x14) as u8;
    w32(PMIF_CHAN + 0x24, 1);
    Some(value)
}

/// An MT6373 LDO: enable, VOSEL and VOCAL registers (hardware addresses,
/// i.e. Linux's mt6373 header values + the regulators node's 0x30) and the
/// VOSEL code of each supported voltage, as read back from Linux on Navi.
struct Ldo {
    name: &'static str,
    en: u16,
    vosel: u16,
    vocal: u16,
    voltages: &'static [(u32, u8)],
}

const MT6373_LDOS: &[Ldo] = &[
    Ldo {
        name: "vfp",
        en: 0x1d15,
        vosel: 0x1e31,
        vocal: 0x1e30,
        voltages: &[(1_800_000, 0x04)],
    },
    Ldo {
        name: "vcn33-2",
        en: 0x1c15,
        vosel: 0x1e15,
        vocal: 0x1e14,
        voltages: &[(3_300_000, 0x0d)],
    },
];

/// Enable an MT6373 LDO at `microvolt` (VOSEL/VOCAL first, as coreboot's
/// mt6373_set_vcn33_3_voltage). Returns whether it was turned on now.
fn enable_ldo(ldo: &Ldo, microvolt: u32) -> Option<bool> {
    let Some(&(_, sel)) = ldo.voltages.iter().find(|(uv, _)| *uv == microvolt) else {
        log::warn!("MT6373 {}: unsupported voltage {microvolt} uV", ldo.name);
        return None;
    };
    let state = spmi_cmd(false, ldo.en, 0)?;
    if state & 1 != 0 {
        return Some(false);
    }
    let (old_sel, old_cal) = (
        spmi_cmd(false, ldo.vosel, 0)?,
        spmi_cmd(false, ldo.vocal, 0)?,
    );
    spmi_cmd(true, ldo.vosel, (old_sel & 0xf0) | sel);
    delay_us(100);
    spmi_cmd(true, ldo.vocal, old_cal & 0xf0);
    delay_us(100);
    spmi_cmd(true, ldo.en, state | 1);
    let on = spmi_cmd(false, ldo.en, 0).is_some_and(|v| v & 1 != 0);
    log::debug!("MT6373 {}: enabled ({on})", ldo.name);
    on.then_some(true)
}

/// Turn on the MT6373 regulators the OS keeps always on (coreboot leaves
/// several off; the touchpad needs VFP and VCN33_2). Returns the names of
/// those turned on now.
fn enable_always_on(regulators: &[super::dt::Regulator]) -> Vec<String> {
    if regulators.is_empty() {
        return Vec::new();
    }
    match spmi_cmd(false, 0x09, 0) {
        Some(id) if id & 0xf0 == 0x70 => {}
        id => {
            log::error!(
                "MT8196 SPMI: slave 5 is not an MT6373 (HWCID {id:x?}); not touching power"
            );
            return Vec::new();
        }
    }
    regulators
        .iter()
        .filter_map(|regulator| {
            let ldo = MT6373_LDOS.iter().find(|ldo| ldo.name == regulator.name)?;
            enable_ldo(ldo, regulator.microvolt)
                .unwrap_or(false)
                .then(|| regulator.name.clone())
        })
        .collect()
}

/// ACPI-style identifier CrabEFI's I2C drivers bind to.
fn driver_id(device: &super::dt::I2cDevice) -> Option<&'static str> {
    if device.is_compatible("google,cr50") || device.is_compatible("google,ti50") {
        Some("GOOG0005")
    } else if device.is_compatible("hid-over-i2c") {
        Some("PNP0C50")
    } else if device.is_compatible("elan,ekth3000") {
        Some("ELAN0000")
    } else {
        None
    }
}

/// Bring up an I2C controller: reuse coreboot's setup when it initialized
/// the bus (TPM), otherwise program it like Linux's i2c-mt65xx.
fn open_bus(bus: &super::dt::I2cBus) -> Option<Controller> {
    if !enable_clocks(bus.base) {
        log::warn!("MT8196 I2C {:#x}: clocks unknown; not used", bus.base);
        return None;
    }
    let buffer = crabefi::efi::allocate_dma_pages(2)?;
    // SAFETY: the board description's controller and APDMA channel, clocked
    // above and Device-mapped; the buffer comes from the NC DMA pool below
    // 4 GiB (mandatory on MT8196) and is owned by this controller.
    let mut controller =
        match unsafe { MtkI2c::new(bus.base as usize, bus.dma as usize, buffer, Delay) } {
            Ok(controller) => controller,
            Err(error) => {
                log::error!("MT8196 I2C {:#x}: {error:?}", bus.base);
                return None;
            }
        };
    if controller.is_configured() {
        return Some(controller);
    }
    configure_pins(&bus.pins);
    let Some(timing) = Timing::for_speed(I2C_CLK_HZ, bus.frequency) else {
        log::error!(
            "MT8196 I2C {:#x}: no timing for {} Hz",
            bus.base,
            bus.frequency
        );
        return None;
    };
    controller.init(timing);
    Some(controller)
}

/// Whether `address` acknowledges (with retries: touchpads NACK while busy).
fn present(bus: &mut Controller, address: u8) -> bool {
    (0..5).any(|_| {
        let ok = bus.transfer(address, &[], &mut [0u8; 1]).is_ok();
        if !ok {
            delay_us(20_000);
        }
        ok
    })
}

/// The board's I2C peripherals CrabEFI drives (TPM, touchpad), from the
/// devicetree when coreboot passed one.
pub fn peripherals() -> Vec<Peripheral> {
    // The APDMA engines do not snoop the caches: bounce pages must be
    // non-cacheable.
    if !crabefi::efi::dma_pool::active() {
        log::warn!("MT8196 I2C: no non-cacheable DMA pool; skipping");
        return Vec::new();
    }
    let board = super::dt::board();
    let mut buses: Vec<_> = board
        .i2c
        .iter()
        .filter(|bus| bus.devices.iter().any(|device| driver_id(device).is_some()))
        .filter_map(|bus| Some((bus, open_bus(bus)?)))
        .collect();
    // Supplies only once the buses idle high.
    let powered = enable_always_on(&board.mt6373_always_on);
    let settle = buses
        .iter()
        .flat_map(|(bus, _)| &bus.devices)
        .filter(|device| {
            device
                .supply
                .as_ref()
                .is_some_and(|supply| powered.contains(supply))
        })
        .map(|device| device.post_power_on_delay_ms)
        .max()
        .unwrap_or(0);
    if settle != 0 {
        crabefi::time::delay_ms(u64::from(settle));
    }

    let mut list = Vec::new();
    for (bus, mut controller) in buses.drain(..) {
        // Peripherals without a supply (TPM) are always there; powered
        // alternatives (second-source touchpads) are probed.
        let devices: Vec<_> = bus
            .devices
            .iter()
            .filter_map(|device| Some((device, driver_id(device)?)))
            .filter(|(device, _)| {
                device.supply.is_none() || present(&mut controller, device.address)
            })
            .collect();
        if devices.is_empty() {
            continue;
        }
        let shared = Rc::new(RefCell::new(controller));
        for (device, id) in devices {
            log::info!("MT8196 I2C {:#x}: {id} at {:#x}", bus.base, device.address);
            list.push(Peripheral::I2c(I2cPeripheral {
                ids: vec![String::from(id)],
                bus: I2cBus::new(RcDevice::new(shared.clone())),
                address: device.address,
                interrupt: device
                    .interrupt
                    .and_then(|(gpio, flags)| interrupt(gpio, flags)),
                hid_descriptor_register: device.hid_descriptor,
            }));
        }
    }
    list
}
