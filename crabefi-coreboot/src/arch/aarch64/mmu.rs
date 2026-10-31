//! Cacheability changes to the identity map built by the AArch64 entry.
//!
//! The entry maps 2 - 3 GB with 2MB blocks when it builds the page tables
//! itself. Devices that read DRAM without snooping the caches, such as the
//! display engine scanning out the coreboot framebuffer on MediaTek SoCs,
//! need their buffers mapped non-cacheable there.

use core::sync::atomic::{AtomicU32, Ordering};

/// Base and size of the range mapped through the 2MB L2 table.
const L2_BASE: u64 = 0x8000_0000;
const L2_SIZE: u64 = 0x4000_0000;
const BLOCK_SIZE: u64 = 0x20_0000;

/// Normal non-cacheable block: AttrIndx=2 (MAIR Attr2 = 0x44), SH=11, AF=1.
const NORMAL_NC_BLOCK: u64 = 0x709;

/// Set by the entry once BSS is zeroed: 1 if the L2 table is in use.
#[unsafe(no_mangle)]
static CRABEFI_EL2_L2_ACTIVE: AtomicU32 = AtomicU32::new(0);

unsafe extern "C" {
    static mut _el2_l2_table: [u64; 512];
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

fn cache_line_size() -> u64 {
    let ctr: u64;
    // SAFETY: CTR_EL0 is readable at EL2 and has no side effects.
    unsafe { core::arch::asm!("mrs {}, ctr_el0", out(reg) ctr) };
    4 << ((ctr >> 16) & 0xf)
}

/// Map `[base, base + size)` as Normal non-cacheable memory.
///
/// Only possible for 2MB-aligned blocks in 2 - 3 GB, and when the entry
/// built the page tables. Returns whether the range was remapped. The range
/// must not be in use, as it is briefly unmapped (break-before-make).
pub fn map_uncached(base: u64, size: u64) -> bool {
    if CRABEFI_EL2_L2_ACTIVE.load(Ordering::Relaxed) == 0 || size == 0 {
        return false;
    }
    let start = base & !(BLOCK_SIZE - 1);
    let Some(end) = base
        .checked_add(size)
        .and_then(|end| end.checked_add(BLOCK_SIZE - 1))
        .map(|end| end & !(BLOCK_SIZE - 1))
    else {
        return false;
    };
    if start < L2_BASE || end > L2_BASE + L2_SIZE {
        return false;
    }

    // Write back and drop any cached lines of the range first.
    let line = cache_line_size();
    let mut addr = start & !(line - 1);
    while addr < end {
        // SAFETY: the range is mapped, cleaning by VA has no other effect.
        unsafe { core::arch::asm!("dc civac, {}", in(reg) addr) };
        addr += line;
    }

    let table = &raw mut _el2_l2_table as *mut u64;
    let first = ((start - L2_BASE) / BLOCK_SIZE) as usize;
    let last = ((end - L2_BASE) / BLOCK_SIZE) as usize;

    // SAFETY: the entries cover only the requested range, which the caller
    // guarantees is unused; the sequence follows break-before-make.
    unsafe {
        for index in first..last {
            core::ptr::write_volatile(table.add(index), 0);
        }
        core::arch::asm!("dsb ishst", "tlbi alle2", "dsb ish", "isb");
        for index in first..last {
            let output = L2_BASE + index as u64 * BLOCK_SIZE;
            core::ptr::write_volatile(table.add(index), output | NORMAL_NC_BLOCK);
        }
        core::arch::asm!("dsb ishst", "isb");
    }

    true
}
