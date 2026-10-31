//! Optional non-cacheable pool for device-visible memory.
//!
//! On platforms whose DMA masters do not snoop the CPU caches (MediaTek
//! MT8196) the platform maps one physical window Normal non-cacheable and
//! registers it here. Rings, contexts, descriptor tables and bounce buffers
//! then come from that window, so drivers that never perform cache
//! maintenance stay correct while all other RAM is cached. Without a
//! registered pool, every allocation falls back to the page allocator.

use crate::cell::Local;

const PAGE: u64 = 4096;
/// Largest supported pool (64 MiB of 4 KiB pages).
const MAX_PAGES: usize = 16384;
const WORDS: usize = MAX_PAGES / 64;

struct Pool {
    base: u64,
    pages: usize,
    used: [u64; WORDS],
}

static POOL: Local<Option<Pool>> = Local::new(None);

impl Pool {
    fn is_used(&self, page: usize) -> bool {
        self.used[page / 64] & (1 << (page % 64)) != 0
    }

    fn set(&mut self, first: usize, count: usize, used: bool) {
        for page in first..first + count {
            if used {
                self.used[page / 64] |= 1 << (page % 64);
            } else {
                self.used[page / 64] &= !(1 << (page % 64));
            }
        }
    }

    fn allocate(&mut self, count: usize) -> Option<u64> {
        let mut run = 0;
        for page in 0..self.pages {
            if self.is_used(page) {
                run = 0;
                continue;
            }
            run += 1;
            if run == count {
                let first = page + 1 - count;
                self.set(first, count, true);
                return Some(self.base + first as u64 * PAGE);
            }
        }
        None
    }
}

/// Register `[base, base + size)` as the platform's non-cacheable DMA pool.
///
/// # Safety
/// The range must be exclusively owned by the caller (allocated from the
/// page allocator for the whole boot), identity mapped Normal non-cacheable,
/// and reachable by every DMA master that receives addresses from it.
pub unsafe fn register(base: u64, size: u64) -> bool {
    let pages = (size / PAGE) as usize;
    if !base.is_multiple_of(PAGE) || pages == 0 || pages > MAX_PAGES {
        return false;
    }
    POOL.with_mut(|pool| {
        *pool = Some(Pool {
            base,
            pages,
            used: [0; WORDS],
        })
    });
    log::info!("DMA pool: {} KiB non-cacheable at {:#x}", size / 1024, base);
    true
}

/// Whether a non-cacheable pool is active.
pub fn active() -> bool {
    POOL.borrow().is_some()
}

/// Allocate `count` zeroed pages from the pool, if one is registered.
pub(crate) fn allocate(count: u64) -> Option<u64> {
    let count = usize::try_from(count).ok().filter(|&count| count != 0)?;
    let address = POOL.with_mut(|pool| pool.as_mut()?.allocate(count))?;
    // SAFETY: the pages were just reserved in the exclusively owned pool.
    unsafe { core::ptr::write_bytes(address as *mut u8, 0, count * PAGE as usize) };
    Some(address)
}

/// Return pages to the pool. Returns false if `address` is not pool memory.
pub(crate) fn free(address: u64, count: u64) -> bool {
    POOL.with_mut(|pool| {
        let Some(pool) = pool.as_mut() else {
            return false;
        };
        let end = pool.base + pool.pages as u64 * PAGE;
        if address < pool.base || address >= end {
            return false;
        }
        let first = ((address - pool.base) / PAGE) as usize;
        let count = (count as usize).min(pool.pages - first);
        pool.set(first, count, false);
        true
    })
}

/// Whether `[address, address + len)` lies inside the pool.
pub fn contains(address: u64, len: u64) -> bool {
    POOL.borrow().as_ref().is_some_and(|pool| {
        address >= pool.base
            && address
                .checked_add(len)
                .is_some_and(|end| end <= pool.base + pool.pages as u64 * PAGE)
    })
}
