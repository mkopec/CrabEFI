//! Primary MT8196 USB host: control and bulk endpoints only (USB storage).
//!
//! Coreboot initializes this controller and its PHY before payload entry.
//! Do not discover/probe the other controllers. Reuse the documented Rauru
//! host-ready GPIO handshake on the pin coreboot names "XHCI init done";
//! do not reconfigure board pinmux or directions.

use crabefi::drivers::usb::{
    self, InitializedController,
    xhci::{XhciController, XhciError},
};
use crabefi::efi::dma::{DmaCoherency, DmaDomain};

use super::gpio;

/// (xHCI host, IPPC) from the board description (`mediatek,mtu3`, host mode).
static IPPC: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

fn read_ippc(offset: u64) -> u32 {
    // SAFETY: the IPPC window of the board's host controller; coreboot
    // initializes the primary USB IPPC before entering the payload.
    unsafe {
        core::ptr::read_volatile(
            (IPPC.load(core::sync::atomic::Ordering::Relaxed) + offset) as *const u32,
        )
    }
}

/// Coreboot's "XHCI init done" GPIO (port, active high), if it published one.
static INIT_DONE: crabefi::cell::Local<Option<(u32, bool)>> = crabefi::cell::Local::new(None);

/// Mirror depthcharge src/board/rauru/board.c: tell the EC that the running
/// xHCI host is ready for VBUS, then wait 500ms before USB enumeration.
fn notify_host_ready() -> Result<(), XhciError> {
    let Some((pin, active_high)) = *INIT_DONE.borrow() else {
        return Ok(());
    };
    // Coreboot (rauru chromeos.c) configures it as a GPIO output, inactive.
    // Only check that; do not change pinmux or direction.
    if !gpio::is_gpio_output(pin) {
        log::error!("MT8196 USB: XHCI init done GPIO{pin} is not a GPIO output");
        return Err(XhciError::NotReady);
    }
    let before = gpio::output(pin);
    gpio::set_output(pin, active_high);
    let after = gpio::output(pin);
    log::info!(
        "MT8196 USB: XHCI init done GPIO{pin} {} -> {}; waiting 500ms for VBUS/device detection",
        u8::from(before),
        u8::from(after)
    );
    if after != active_high {
        return Err(XhciError::NotReady);
    }
    crabefi::time::delay_ms(500);
    Ok(())
}

/// Use coreboot's "XHCI init done" GPIO (LB_TAG_GPIO) for the handshake.
pub fn set_init_done_gpio(gpio: Option<crate::tables::GpioInfo>) {
    if gpio.is_none() {
        log::info!("MT8196 USB: no \"XHCI init done\" GPIO from coreboot; no VBUS handshake");
    }
    *INIT_DONE.borrow_mut() = gpio.map(|gpio| (gpio.port, gpio.active_high));
}

pub fn init() {
    let Some((host, ippc)) = super::dt::board().xhci else {
        log::info!("MT8196 USB: no host controller in the board description");
        return;
    };
    IPPC.store(ippc, core::sync::atomic::Ordering::Relaxed);
    if !crabefi::efi::dma_pool::active() {
        log::error!("MT8196 USB disabled: no non-cacheable DMA pool");
        return;
    }

    let power = read_ippc(0x04);
    let status1 = read_ippc(0x10);
    let status2 = read_ippc(0x14);
    let capability = read_ippc(0x24);
    log::info!(
        "MT8196 USB IPPC: power={power:#x}, status={status1:#x}/{status2:#x}, cap={capability:#x}"
    );
    let u3_ports = capability & 0xff;
    let u2_ports = (capability >> 8) & 0xff;
    let required = 1 | (1 << 8) | (1 << 10) | if u3_ports != 0 { 1 << 16 } else { 0 };
    if power & 1 != 0
        || status1 & required != required
        || status2 & 1 == 0
        || u2_ports == 0
        || u2_ports > 6
        || u3_ports > 4
    {
        log::error!("MT8196 USB disabled: coreboot did not leave the primary host ready");
        return;
    }

    log::info!("MT8196 USB: storage only (no periodic endpoints, so no USB HID)");
    let domain = DmaDomain {
        cpu_base: 0x8000_0000,
        device_base: 0x8000_0000,
        size: 0x8000_0000,
        // Effective cache ownership, NOT hardware snooping: every buffer the
        // controller sees comes from the non-cacheable DMA pool (DmaBuffer
        // refuses other memory while the pool is active).
        coherency: DmaCoherency::Coherent,
    };
    // SAFETY: the entry maps the primary host as Device; every buffer the
    // controller receives (rings, contexts, control and bulk bounce
    // buffers) comes from the non-cacheable DMA pool below 4 GiB.
    // Coreboot initializes the primary host's clocks, PHY and host role;
    // its status was checked above. Linux's MT8196 primary USB node has no
    // IOMMU attachment. Periodic endpoints are explicitly disabled.
    let controller = crabefi::exec::block_on(unsafe {
        XhciController::new_mmio_bulk_only_with_ready(host, domain, notify_host_ready)
    });
    match controller {
        Ok(controller) => {
            if usb::register_mmio_controller(
                host,
                InitializedController::Xhci(alloc::boxed::Box::new(controller)),
            )
            .is_err()
            {
                log::error!("MT8196 USB: controller registration failed");
            }
        }
        Err(error) => log::error!("MT8196 USB: controller initialization failed: {error:?}"),
    }
}
