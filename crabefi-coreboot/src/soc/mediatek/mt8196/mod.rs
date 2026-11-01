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
#[cfg(feature = "mt8196")]
pub mod nor;
#[cfg(target_arch = "aarch64")]
mod ufs;
#[cfg(target_arch = "aarch64")]
mod usb;

#[cfg(target_arch = "aarch64")]
pub use self::platform::*;

#[cfg(target_arch = "aarch64")]
mod platform {
    use alloc::boxed::Box;

    use super::{dt, ec, i2c, nor, ufs, usb};
    use crate::tables::CorebootInfo;

    /// Whether the runtime image writes the variable store on the NOR.
    static RUNTIME_NOR: crabefi::cell::LocalCell<bool> = crabefi::cell::LocalCell::new(false);

    /// The DMA masters (xHCI, UFS, I2C APDMA) only reach the low 4 GiB:
    /// keep firmware allocations low (high RAM stays in the map for the OS).
    /// Runs after the page allocator is up and before the heap.
    pub fn init_memory() {
        crabefi::efi::allocator::set_any_pages_preferred_max(0xffff_ffff);
    }

    /// Describe the board and attach its I2C peripherals, variable store and
    /// capsule flash access. Runs after the heap is up (the FMAP lookup and
    /// the devicetree parse allocate), before init_platform().
    pub fn configure(
        cb_info: &CorebootInfo,
        config: &mut crabefi::PlatformConfig<'_>,
        capsule_storage: &mut Option<Box<dyn crabefi::FirmwareStorage>>,
    ) {
        dt::init(cb_info.devicetree.map(|(address, size)| {
            // SAFETY: coreboot's (or the FIT handoff's copied) devicetree.
            unsafe { core::slice::from_raw_parts(address as *const u8, size as usize) }
        }));
        usb::set_init_done_gpio(cb_info.gpio("XHCI init done"));
        // TPM and touchpad: bound with the other peripherals before measured
        // boot is set up, so the TPM backs the TCG protocols.
        config.peripherals.extend(i2c::peripherals());

        // The NOR controller is not memory-mapped flash. Variable storage
        // gets a bounded region; the capsule backend shares the controller.
        if cb_info.boot_media.is_none() {
            log::warn!("MT8196: coreboot tables have no boot media record");
        }
        config.variable_storage = match cb_info
            .boot_media
            .and_then(|media| nor::variable_region(media.boot_media_size, Some(media.fmap_offset)))
        {
            Some(region) => {
                // Runtime SetVariable writes this flash in place; the OS's
                // devicetree gets the controller reserved (fixup_devicetree).
                (config.runtime.nvram, config.runtime.external_ranges) = region.runtime_config();
                RUNTIME_NOR.set(true);
                *capsule_storage = Some(region.firmware_storage());
                crabefi::VariableStorage::Platform(region)
            }
            None => crabefi::VariableStorage::None,
        };
    }

    /// Keep the OS off the NOR controller the runtime image writes
    /// (PlatformHooks::fixup_devicetree).
    pub fn fixup_devicetree(tree: &mut crabefi::fdt_tree::Tree) {
        if RUNTIME_NOR.get() {
            match dt::reserve_nor(tree) {
                0 => log::warn!("Devicetree fixup: no SPI NOR controller node to reserve"),
                _ => log::info!("Devicetree fixup: SPI NOR controller reserved for UEFI variables"),
            }
        }
    }

    /// Bring up the boot devices (PlatformHooks::init_devices).
    pub fn init_devices() {
        ec::init();
        ufs::init();
        usb::init();
    }
}
