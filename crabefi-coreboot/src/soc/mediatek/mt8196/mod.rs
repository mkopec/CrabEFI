//! MediaTek MT8196: board devices from the devicetree coreboot passes, on
//! the SoC's controllers.
//!
//! The payload calls [`init_memory`] and [`init_devices`] in boot order;
//! nothing here is board-specific. The UART and the non-cacheable DMA pool
//! are set up from the devicetree by the payload.

#[cfg(target_arch = "aarch64")]
pub use self::platform::*;

#[cfg(target_arch = "aarch64")]
mod platform {
    /// The DMA masters (xHCI, UFS, I2C APDMA) only reach the low 4 GiB:
    /// keep firmware allocations low (high RAM stays in the map for the OS).
    /// Runs after the page allocator is up and before the heap.
    pub fn init_memory() {
        crabefi::efi::allocator::set_any_pages_preferred_max(0xffff_ffff);
    }

    /// Bring up the boot devices (PlatformHooks::init_devices).
    pub fn init_devices() {}
}
