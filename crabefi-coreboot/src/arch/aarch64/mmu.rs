//! Checks of the identity map built by the AArch64 entry.

unsafe extern "C" {
    static _el2_l0_table: u8;
}

/// Warn when the entry's page tables map memory below the lowest RAM under
/// 4 GiB as Normal: those are device registers, which must not be mapped
/// cacheable (speculative accesses). The build's LOW_DEVICE_GB sets how many
/// gigabytes from address 0 are Device.
pub fn check_low_device_range(lowest_ram: u64) {
    let ttbr0: u64;
    // SAFETY: TTBR0_EL2 is readable at EL2 and has no side effects.
    unsafe { core::arch::asm!("mrs {}, ttbr0_el2", out(reg) ttbr0) };
    // Only the tables the entry built follow LOW_DEVICE_GB.
    if ttbr0 & !0xfff != core::ptr::addr_of!(_el2_l0_table) as u64 {
        return;
    }
    // LOW_DEVICE_GB is an absolute linker symbol: load its value from a
    // literal (an adrp to it is out of range for payloads above 4 GiB).
    let low_device_gb: u64;
    // SAFETY: loads a link-time constant; no memory is accessed.
    unsafe { core::arch::asm!("ldr {}, =LOW_DEVICE_GB", out(reg) low_device_gb) };
    let device_end = low_device_gb << 30;
    let ram_block = lowest_ram & !0x3fff_ffff;
    if lowest_ram < 1 << 32 && device_end < ram_block {
        log::warn!(
            "{device_end:#x}..{ram_block:#x} is below RAM but mapped Normal; \
             build with LOW_DEVICE_GB={}",
            ram_block >> 30
        );
    }
}
