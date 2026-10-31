// SPDX-License-Identifier: MIT OR Apache-2.0
//! MediaTek MT65xx I2C master with its APDMA channel, polled.
//!
//! Ported from coreboot's soc/mediatek/common/i2c.c (controllers firmware
//! already set up) and Linux's i2c-mt65xx (controllers programmed here, as
//! for mt8188-compatible parts). Data moves through a caller-provided bounce
//! buffer; one write, one read or a write-then-read per controller
//! transaction.

use embedded_hal::{
    delay::DelayNs,
    i2c::{self, ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation, SevenBitAddress},
};
use tock_registers::{
    interfaces::{Readable, Writeable},
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite},
};

register_bitfields![u32,
    Intr [
        TRANSAC_COMP OFFSET(0) NUMBITS(1) [],
        ACKERR OFFSET(1) NUMBITS(1) [],
        HS_NACKERR OFFSET(2) NUMBITS(1) [],
        ARB_LOST OFFSET(3) NUMBITS(1) [],
        RS_TRANSFER OFFSET(4) NUMBITS(1) []
    ],
    Control [
        REPEATED_START OFFSET(1) NUMBITS(1) [],
        DMA_ENABLE OFFSET(2) NUMBITS(1) [],
        CLK_EXT OFFSET(3) NUMBITS(1) [],
        DIR_CHG OFFSET(4) NUMBITS(1) [],
        ACK_ERR_DET_EN OFFSET(5) NUMBITS(1) [],
        DMAACK_EN OFFSET(8) NUMBITS(1) [],
        ASYNC_MODE OFFSET(9) NUMBITS(1) []
    ],
    DmaCon [
        RX OFFSET(0) NUMBITS(1) [],
        ASYNC OFFSET(2) NUMBITS(1) [],
        SKIP_CONFIG OFFSET(4) NUMBITS(1) [],
        DIR_CHANGE OFFSET(9) NUMBITS(1) []
    ]
];

register_structs! {
    Regs {
        (0x00 => _reserved0),
        (0x08 => intr_mask: ReadWrite<u32, Intr::Register>),
        (0x0c => intr_stat: ReadWrite<u32, Intr::Register>),
        (0x10 => control: ReadWrite<u32, Control::Register>),
        (0x14 => transfer_len: ReadWrite<u32>),
        (0x18 => transac_len: ReadWrite<u32>),
        (0x1c => delay_len: ReadWrite<u32>),
        (0x20 => timing: ReadWrite<u32>),
        (0x24 => start: ReadWrite<u32>),
        (0x28 => ext_conf: ReadWrite<u32>),
        (0x2c => ltiming: ReadWrite<u32>),
        (0x30 => hs: ReadWrite<u32>),
        (0x34 => io_config: ReadWrite<u32>),
        (0x38 => fifo_addr_clr: ReadWrite<u32>),
        (0x3c => sda_timing: ReadWrite<u32>),
        (0x40 => _reserved1),
        (0x44 => transfer_aux_len: ReadWrite<u32>),
        (0x48 => clock_div: ReadWrite<u32>),
        (0x4c => _reserved2),
        (0x50 => softreset: ReadWrite<u32>),
        (0x54 => _reserved3),
        (0x90 => scl_mis_comp: ReadWrite<u32>),
        (0x94 => slave_addr: ReadWrite<u32>),
        (0x98 => _reserved4),
        (0xe4 => debug_stat: ReadOnly<u32>),
        (0xe8 => @END),
    }
}

register_structs! {
    DmaRegs {
        (0x00 => int_flag: ReadWrite<u32>),
        (0x04 => _reserved0),
        (0x08 => en: ReadWrite<u32>),
        (0x0c => rst: ReadWrite<u32>),
        (0x10 => _reserved1),
        (0x18 => con: ReadWrite<u32, DmaCon::Register>),
        (0x1c => tx_mem_addr: ReadWrite<u32>),
        (0x20 => rx_mem_addr: ReadWrite<u32>),
        (0x24 => tx_len: ReadWrite<u32>),
        (0x28 => rx_len: ReadWrite<u32>),
        (0x2c => _reserved2),
        (0x54 => tx_4g: ReadWrite<u32>),
        (0x58 => rx_4g: ReadWrite<u32>),
        (0x5c => @END),
    }
}

/// START: transfer start, and multi-transfer trigger with auto-restart.
const START_EN: u32 = 1;
const RS_MUL_TRIG: u32 = 1 << 14;
/// SOFTRESET: soft reset and APDMA handshake.
const SOFTRESET_ALL: u32 = 1 | 0x20;
/// DMA_RST: warm reset, then hard reset with handshake.
const DMA_WARM_RST: u32 = 1;
const DMA_HARD_RST: u32 = 2 | 4;
const IO_OPEN_DRAIN: u32 = 3;
const TRANSFER_TIMEOUT_US: u32 = 500_000;

/// Bus timing register values, in the order Linux programs them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Timing {
    pub clock_div: u32,
    pub htiming: u32,
    pub ltiming: u32,
    pub hs: u32,
    pub ext: u32,
    /// SDA_TIMING (Linux only; coreboot never writes it).
    pub sda: u32,
}

struct Spec {
    min_low_ns: u32,
    min_su_sta_ns: u32,
    max_hd_dat_ns: u32,
    min_su_dat_ns: u32,
}

fn spec(speed: u32) -> Spec {
    if speed <= 100_000 {
        let b = 1000 / 3;
        Spec {
            min_low_ns: 4700 + b,
            min_su_sta_ns: 4700 + b,
            max_hd_dat_ns: 3450 - b,
            min_su_dat_ns: 250 + b,
        }
    } else if speed <= 400_000 {
        let b = 300 / 3;
        Spec {
            min_low_ns: 1300 + b,
            min_su_sta_ns: 600 + b,
            max_hd_dat_ns: 900 - b,
            min_su_dat_ns: 100 + b,
        }
    } else {
        let b = 20 / 3;
        Spec {
            min_low_ns: 500 + b,
            min_su_sta_ns: 260 + b,
            max_hd_dat_ns: 400 - b,
            min_su_dat_ns: 50 + b,
        }
    }
}

/// coreboot check_ac_timing().
fn check_ac_timing(clk_src: u32, speed: u32, step: u32, sample: u32, t: &mut Timing) -> bool {
    let s = spec(speed);
    let sample_ns = (1_000_000_000u64 * u64::from(sample + 1) / u64::from(clk_src)) as u32;
    let clk_ns = 1_000_000_000 / clk_src;
    let su_sta = s.min_su_sta_ns.div_ceil(clk_ns);
    if su_sta > 0x100 {
        return false;
    }
    let mut low = s.min_low_ns.div_ceil(sample_ns);
    let high;
    if 2 * step > low && low < 64 {
        if low > step {
            high = 2 * step - low;
        } else {
            high = step;
            low = step;
        }
    } else {
        return false;
    }
    let mut sda_max = s.max_hd_dat_ns / sample_ns;
    if sda_max > low {
        sda_max = 0;
    }
    let mut sda_min = s.min_su_dat_ns.div_ceil(sample_ns);
    if sda_min < low {
        sda_min = 0;
    }
    if sda_min > sda_max {
        return false;
    }
    t.htiming = (sample << 8) | high;
    t.ltiming = (sample << 6) | low;
    t.ext = (su_sta << 8) | 1;
    t.sda = (1 << 12) | ((sda_max + sda_min) / 2);
    true
}

/// coreboot calculate_speed().
fn calculate_speed(clk_src: u32, target: u32, t: &mut Timing) -> bool {
    let clk_div = t.clock_div;
    let opt_div = (clk_src >> 1).div_ceil(target) as i64;
    let mut best_mul = 8 * 64i64;
    let (mut base_sample, mut base_step) = (8i64, 64i64);
    let mut constraint = 0i64;
    let mut success = false;
    for sample in 1..=8i64 {
        constraint = if sample == 1 {
            i64::from(clk_div != 0)
        } else if clk_div > 1 {
            1
        } else if clk_div == 0 {
            -1
        } else {
            0
        };
        let step = (opt_div + constraint + sample - 1) / sample;
        if step > 64 {
            continue;
        }
        let mul = step * sample;
        if mul >= best_mul {
            continue;
        }
        if !check_ac_timing(clk_src, target, (step - 1) as u32, (sample - 1) as u32, t) {
            continue;
        }
        success = true;
        best_mul = mul;
        base_sample = sample;
        base_step = step;
        if best_mul == opt_div + constraint {
            break;
        }
    }
    if !success {
        return false;
    }
    let divisor = 2 * (base_sample * base_step - constraint);
    divisor > 0 && i64::from(clk_src) / divisor <= i64::from(target)
}

impl Timing {
    /// Timing for `speed_hz` up to Fast-mode Plus (no high-speed mode) from
    /// a `clock_hz` source clock (coreboot mtk_i2c_speed_init()).
    pub fn for_speed(clock_hz: u32, speed_hz: u32) -> Option<Self> {
        if speed_hz == 0 {
            return None;
        }
        (1..=32u32).find_map(|clk_div| {
            let mut t = Timing {
                clock_div: clk_div - 1,
                ..Timing::default()
            };
            calculate_speed(clock_hz / clk_div, speed_hz, &mut t).then_some(t)
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidRegion,
    /// The bounce buffer is too small or not below 4 GiB.
    InvalidBuffer,
    /// Empty or longer than half the bounce buffer.
    InvalidLength,
    NoAcknowledge,
    ArbitrationLoss,
    /// No completion: INTR_STAT and DEBUG_STAT at the timeout.
    Timeout {
        status: u32,
        debug: u32,
    },
}

impl i2c::Error for Error {
    fn kind(&self) -> ErrorKind {
        match *self {
            Self::NoAcknowledge => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            Self::ArbitrationLoss => ErrorKind::ArbitrationLoss,
            Self::InvalidRegion
            | Self::InvalidBuffer
            | Self::InvalidLength
            | Self::Timeout { .. } => ErrorKind::Other,
        }
    }
}

/// Complete bounce buffer writes before the device reads them.
fn publish() {
    #[cfg(target_arch = "aarch64")]
    // SAFETY: a barrier instruction only.
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags))
    };
    #[cfg(not(target_arch = "aarch64"))]
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
}

pub struct MtkI2c<D> {
    regs: &'static Regs,
    dma: &'static DmaRegs,
    tx: &'static mut [u8],
    rx: &'static mut [u8],
    delay: D,
    /// Set by [`MtkI2c::init`]: run transfers as Linux's i2c-mt65xx does.
    linux: Option<Timing>,
}

impl<D: DelayNs> MtkI2c<D> {
    /// Wrap a controller and its APDMA channel. Transfers keep the timing
    /// firmware programmed until [`MtkI2c::init`] is called. Each direction
    /// gets half of `buffer`, which limits the transfer length.
    ///
    /// # Safety
    /// `base` and `dma` are the controller's and its APDMA channel's register
    /// blocks, clocked and Device-mapped, used by nothing else while this
    /// driver exists. `buffer` is identity-mapped, DMA-coherent memory (for
    /// example non-cacheable) below 4 GiB.
    pub unsafe fn new(
        base: usize,
        dma: usize,
        buffer: &'static mut [u8],
        delay: D,
    ) -> Result<Self, Error> {
        if base == 0 || dma == 0 || base & 3 != 0 || dma & 3 != 0 {
            return Err(Error::InvalidRegion);
        }
        let end = (buffer.as_ptr() as usize).checked_add(buffer.len());
        if buffer.len() < 2 || end.is_none_or(|end| end > 1 << 32) {
            return Err(Error::InvalidBuffer);
        }
        let (tx, rx) = buffer.split_at_mut(buffer.len() / 2);
        Ok(Self {
            // SAFETY: the caller's hardware contract for both blocks.
            regs: unsafe { &*(base as *const Regs) },
            dma: unsafe { &*(dma as *const DmaRegs) },
            tx,
            rx,
            delay,
            linux: None,
        })
    }

    /// Whether firmware programmed the bus timing.
    pub fn is_configured(&self) -> bool {
        self.regs.timing.get() != 0
    }

    /// Program the controller with `timing` (Linux mtk_i2c_init_hw() for
    /// mt8188_compat: APDMA sync, timing and ltiming adjust, open drain).
    pub fn init(&mut self, timing: Timing) {
        self.linux = Some(timing);
        self.init_hw(timing);
    }

    fn init_hw(&mut self, t: Timing) {
        let (regs, dma) = (self.regs, self.dma);
        regs.start.set(0);
        regs.intr_stat.set(regs.intr_stat.get());
        dma.rst.set(DMA_WARM_RST);
        self.delay.delay_us(10);
        dma.rst.set(0);
        self.delay.delay_us(10);
        dma.rst.set(DMA_HARD_RST);
        regs.softreset.set(SOFTRESET_ALL);
        self.delay.delay_us(10);
        dma.rst.set(0);
        regs.softreset.set(0);
        regs.io_config.set(IO_OPEN_DRAIN);
        regs.timing.set(t.htiming);
        regs.hs.set(t.hs);
        regs.ltiming.set(t.ltiming);
        regs.clock_div.set(t.clock_div);
        regs.scl_mis_comp.set(0);
        regs.sda_timing.set(t.sda);
        regs.ext_conf.set(t.ext);
        regs.control
            .write(Control::ACK_ERR_DET_EN::SET + Control::CLK_EXT::SET + Control::DMA_ENABLE::SET);
        regs.delay_len.set(2);
    }

    /// coreboot i2c_hw_reset() for I2C_APDMA_ASYNC controllers.
    fn reset(&mut self) {
        let (regs, dma) = (self.regs, self.dma);
        dma.rst.set(DMA_WARM_RST);
        self.delay.delay_us(10);
        dma.rst.set(0);
        self.delay.delay_us(10);
        dma.rst.set(DMA_HARD_RST);
        regs.softreset.set(SOFTRESET_ALL);
        self.delay.delay_us(10);
        dma.rst.set(0);
        regs.softreset.set(0);
    }

    fn stage(&mut self, write: &[u8], read_len: usize) {
        self.tx[..write.len()].copy_from_slice(write);
        let _ = read_len;
    }

    fn finish_read(&self, read: &mut [u8]) {
        let len = read.len();
        read.copy_from_slice(&self.rx[..len]);
    }

    /// Poll INTR_STAT until `done` returns a result (accumulating status).
    fn wait(
        &mut self,
        accumulate: bool,
        done: impl Fn(u32) -> Option<Result<(), Error>>,
    ) -> Result<(), Error> {
        let mut seen = 0;
        for _ in 0..TRANSFER_TIMEOUT_US {
            let status = self.regs.intr_stat.get();
            seen = if accumulate { seen | status } else { status };
            if let Some(result) = done(seen) {
                return result;
            }
            self.delay.delay_us(1);
        }
        Err(Error::Timeout {
            status: self.regs.intr_stat.get(),
            debug: self.regs.debug_stat.get(),
        })
    }

    /// Linux mtk_i2c_transfer()/do_transfer(), polled.
    fn transfer_linux(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        let (wl, rl) = (write.len(), read.len());
        let (regs, dma) = (self.regs, self.dma);
        let wrrd = wl != 0 && rl != 0;
        // WRRD disables auto-restart; single messages use it (mt8188: auto_restart=1).
        let restart = if wrrd {
            0
        } else {
            Intr::RS_TRANSFER::SET.value
        };
        let mut control = regs.control.get()
            & !(Control::DIR_CHG::SET.value | Control::REPEATED_START::SET.value);
        if wrrd {
            control |= Control::DIR_CHG::SET.value | Control::REPEATED_START::SET.value;
        }
        regs.control.set(control);
        regs.slave_addr
            .set(u32::from(address) << 1 | u32::from(wl == 0));
        let errors = (Intr::ACKERR::SET + Intr::HS_NACKERR::SET + Intr::ARB_LOST::SET).value;
        let events = restart | errors | Intr::TRANSAC_COMP::SET.value;
        regs.intr_stat.set(events);
        regs.fifo_addr_clr.set(1);
        regs.intr_mask.set(events);
        if wrrd {
            regs.transfer_len.set(wl as u32);
            regs.transfer_aux_len.set(rl as u32);
            regs.transac_len.set(2);
        } else {
            regs.transfer_len.set(wl.max(rl) as u32);
            regs.transac_len.set(1);
        }
        self.stage(write, rl);
        let sync = DmaCon::SKIP_CONFIG::SET + DmaCon::ASYNC::SET;
        dma.int_flag.set(0);
        if wrrd {
            dma.con.write(sync + DmaCon::DIR_CHANGE::SET);
        } else if rl != 0 {
            dma.con.write(sync + DmaCon::RX::SET);
        } else {
            dma.con.write(sync);
        }
        dma.tx_4g.set(0);
        dma.rx_4g.set(0);
        if wl != 0 {
            dma.tx_mem_addr.set(self.tx.as_ptr() as u32);
            dma.tx_len.set(wl as u32);
        }
        if rl != 0 {
            dma.rx_mem_addr.set(self.rx.as_ptr() as u32);
            dma.rx_len.set(rl as u32);
        }
        publish();
        dma.en.set(1);
        regs.start.set(if restart != 0 {
            START_EN | RS_MUL_TRIG
        } else {
            START_EN
        });

        let complete = Intr::TRANSAC_COMP::SET.value | restart;
        let nack = (Intr::ACKERR::SET + Intr::HS_NACKERR::SET).value;
        let result = self.wait(true, |seen| {
            if seen & nack != 0 {
                Some(Err(Error::NoAcknowledge))
            } else if seen & Intr::ARB_LOST::SET.value != 0 {
                Some(Err(Error::ArbitrationLoss))
            } else if seen & complete != 0 {
                Some(Ok(()))
            } else {
                None
            }
        });
        regs.intr_stat.set(events);
        regs.intr_mask.set(0);
        if result.is_ok() && rl != 0 {
            self.finish_read(read);
        }
        if result.is_err() {
            // Linux re-initializes the controller after a failed transfer.
            if let Some(t) = self.linux {
                self.init_hw(t);
            }
        }
        result
    }

    /// coreboot mtk_i2c_transfer() on a firmware-configured controller.
    fn transfer_firmware(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Error> {
        let (wl, rl) = (write.len(), read.len());
        let (regs, dma) = (self.regs, self.dma);
        let events = Intr::TRANSAC_COMP::SET + Intr::ACKERR::SET + Intr::HS_NACKERR::SET;
        regs.intr_stat.write(events);
        regs.fifo_addr_clr.set(1);
        regs.intr_mask.write(events);
        let control = Control::ASYNC_MODE::SET
            + Control::DMAACK_EN::SET
            + Control::ACK_ERR_DET_EN::SET
            + Control::DMA_ENABLE::SET
            + Control::CLK_EXT::SET
            + Control::REPEATED_START::SET;
        let sync = DmaCon::SKIP_CONFIG::SET + DmaCon::ASYNC::SET;
        self.stage(write, rl);
        let (tx, rx) = (self.tx.as_ptr() as u32, self.rx.as_ptr() as u32);
        match (wl, rl) {
            (_, 0) => {
                regs.control.write(control);
                regs.transac_len.set(1);
                regs.transfer_len.set(wl as u32);
                regs.slave_addr.set(u32::from(address) << 1);
                dma.con.write(sync);
                dma.tx_mem_addr.set(tx);
                dma.tx_len.set(wl as u32);
            }
            (0, _) => {
                regs.control.write(control);
                regs.transac_len.set(1);
                regs.transfer_len.set(rl as u32);
                regs.slave_addr.set(u32::from(address) << 1 | 1);
                dma.con.write(sync + DmaCon::RX::SET);
                dma.rx_mem_addr.set(rx);
                dma.rx_len.set(rl as u32);
            }
            _ => {
                regs.control.write(control + Control::DIR_CHG::SET);
                regs.transfer_len.set(wl as u32);
                regs.transfer_aux_len.set(rl as u32);
                regs.transac_len.set(2);
                regs.slave_addr.set(u32::from(address) << 1);
                dma.con.write(sync + DmaCon::DIR_CHANGE::SET);
                dma.tx_mem_addr.set(tx);
                dma.tx_len.set(wl as u32);
                dma.rx_mem_addr.set(rx);
                dma.rx_len.set(rl as u32);
            }
        }
        publish();
        dma.int_flag.set(0);
        dma.en.set(1);
        regs.start.set(START_EN);

        let nack = (Intr::ACKERR::SET + Intr::HS_NACKERR::SET).value;
        let result = self.wait(false, |status| {
            if status & nack != 0 {
                Some(Err(Error::NoAcknowledge))
            } else if status & Intr::TRANSAC_COMP::SET.value != 0 {
                Some(Ok(()))
            } else {
                None
            }
        });
        if result.is_ok() && rl != 0 {
            self.finish_read(read);
        }
        regs.intr_stat.write(events);
        regs.intr_mask.write(events);
        self.reset();
        result
    }

    /// One write, one read, or a write-then-read with a repeated start.
    pub fn transfer(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        let (wl, rl) = (write.len(), read.len());
        if wl > self.tx.len() || rl > self.rx.len() || (wl == 0 && rl == 0) {
            return Err(Error::InvalidLength);
        }
        if self.linux.is_some() {
            self.transfer_linux(address, write, read)
        } else {
            self.transfer_firmware(address, write, read)
        }
    }
}

impl<D> ErrorType for MtkI2c<D> {
    type Error = Error;
}

impl<D: DelayNs> I2c<SevenBitAddress> for MtkI2c<D> {
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        let mut index = 0;
        while let Some((first, rest)) = operations.get_mut(index..).and_then(<[_]>::split_first_mut)
        {
            match first {
                // A write followed by a read uses a repeated start.
                Operation::Write(write) => {
                    if let Some(Operation::Read(read)) = rest.first_mut() {
                        self.transfer(address, write, read)?;
                        index += 2;
                        continue;
                    }
                    if !write.is_empty() {
                        self.transfer(address, write, &mut [])?;
                    }
                }
                Operation::Read(read) => {
                    if !read.is_empty() {
                        self.transfer(address, &[], read)?;
                    }
                }
            }
            index += 1;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::Timing;

    #[test]
    fn timing_400khz_from_mt8196_clock() {
        // Values the touchpad bus (I2C0, 400 kHz) has been running with.
        assert_eq!(
            Timing::for_speed(124_800_000, 400_000),
            Some(Timing {
                clock_div: 0,
                htiming: 0x419,
                ltiming: 0x123,
                hs: 0,
                ext: 0x5801,
                sda: 0x100a,
            })
        );
    }

    #[test]
    fn timing_rejects_zero_speed() {
        assert_eq!(Timing::for_speed(124_800_000, 0), None);
        assert!(Timing::for_speed(124_800_000, 100_000).is_some());
        assert!(Timing::for_speed(124_800_000, 1_000_000).is_some());
    }
}
