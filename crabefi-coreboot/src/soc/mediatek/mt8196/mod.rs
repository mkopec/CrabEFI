//! MediaTek MT8196: board devices from the devicetree coreboot passes, on
//! the SoC's controllers.
//!
//! The payload calls [`init_memory`], [`configure`] and [`init_devices`] in
//! boot order; nothing here is board-specific. Board wiring comes from the
//! devicetree and from coreboot's named GPIOs; the UART and the
//! non-cacheable DMA pool are set up from the devicetree by the payload.

pub mod dt;
#[cfg(target_arch = "aarch64")]
mod ec;
#[cfg(target_arch = "aarch64")]
mod gpio;
#[cfg(target_arch = "aarch64")]
mod i2c;
#[cfg(target_arch = "aarch64")]
mod ufs;
#[cfg(target_arch = "aarch64")]
mod usb;

#[cfg(target_arch = "aarch64")]
pub use self::platform::*;

#[cfg(target_arch = "aarch64")]
mod platform {
    use super::{dt, ec, i2c, ufs, usb};
    use crate::tables::CorebootInfo;

    /// The DMA masters (xHCI, UFS, I2C APDMA) only reach the low 4 GiB:
    /// keep firmware allocations low (high RAM stays in the map for the OS).
    /// Runs after the page allocator is up and before the heap.
    pub fn init_memory() {
        crabefi::efi::allocator::set_any_pages_preferred_max(0xffff_ffff);
    }

    /// Describe the board and attach its I2C peripherals. Runs after the
    /// heap is up (the devicetree parse allocates), before init_platform().
    pub fn configure(cb_info: &CorebootInfo, config: &mut crabefi::PlatformConfig<'_>) {
        dt::init(cb_info.devicetree.map(|(address, size)| {
            // SAFETY: coreboot's (or the FIT handoff's copied) devicetree.
            unsafe { core::slice::from_raw_parts(address as *const u8, size as usize) }
        }));
        usb::set_init_done_gpio(cb_info.gpio("XHCI init done"));
        // TPM and touchpad: bound with the other peripherals before measured
        // boot is set up, so the TPM backs the TCG protocols.
        config.peripherals.extend(i2c::peripherals());
    }

    /// Bring up the boot devices (PlatformHooks::init_devices).
    pub fn init_devices() {
        ec::init();
        ufs::init();
        usb::init();
    }
}
