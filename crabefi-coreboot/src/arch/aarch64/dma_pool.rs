//! Non-cacheable DMA pool for devices that do not snoop the CPU caches.
//!
//! Carved out of RAM in the 2 - 3 GB window, where the entry maps 2MB blocks
//! that map_uncached() can re-attribute.

/// Register a 16 MiB non-cacheable pool for DMA buffers
/// (crabefi::efi::allocate_dma_pages). Runs after the page allocator is up.
pub fn register() {
    use crabefi::efi::allocator::{AllocateType, MemoryType, allocate_pages};
    const BLOCK: u64 = 0x20_0000;
    const SIZE: u64 = 16 << 20;
    let mut address = 0xbfff_ffff;
    let status = allocate_pages(
        AllocateType::AllocateMaxAddress,
        MemoryType::BootServicesData,
        (SIZE + BLOCK) / 4096,
        &mut address,
    );
    let base = address.next_multiple_of(BLOCK);
    if status != r_efi::efi::Status::SUCCESS || !crate::arch_mmu::map_uncached(base, SIZE) {
        log::error!(
            "No non-cacheable DMA pool ({status:?} at {address:#x}): \
             devices that need one are not used"
        );
        return;
    }
    // SAFETY: the block-aligned range lies inside a boot-lifetime allocation
    // that nothing else uses, now mapped Normal non-cacheable.
    unsafe { crabefi::efi::dma_pool::register(base, SIZE) };
}
