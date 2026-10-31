//! MT8196 SPI NOR controller (SFLASH) and the FMAP variable region (`NVRAM`).
//!
//! The controller is crabefi-hw's MediaTek NOR driver: DMA reads (dual I/O,
//! as coreboot's CONFIG_FLASH_DUAL_IO_READ) through a bounce buffer in the
//! payload image, buffered page program and 4 KiB/64 KiB erase. Pads,
//! clocks and read timing are left as coreboot's bootblock configured them.
//!
//! The variable store receives only the bounded FMAP variable region (`NVRAM`,
//! `SMMSTORE` or `RW_NVRAM`).

use alloc::boxed::Box;
use crabefi::{FirmwareStorage, StorageBackend, StorageError};
use crabefi_hw::mediatek::nor::{self, MtkNor};

/// Clock controllers holding the flash clock gates (runtime ungating).
const TOPCKGEN: u64 = 0x1000_0000;
const PERICFG_AO: u64 = 0x1664_0000;

pub const SECTOR: u32 = nor::SECTOR;
/// Bounce buffer for DMA reads, below 4 GiB.
const DMA_LEN: usize = 0x4000;

/// DMA bounce buffer inside the payload image (below 4 GiB): the region is
/// located before the EFI page allocator exists.
#[repr(C, align(4096))]
struct DmaBuffer([u8; DMA_LEN]);
static mut DMA_BUFFER: DmaBuffer = DmaBuffer([0; DMA_LEN]);

fn storage_error(error: nor::Error) -> StorageError {
    match error {
        nor::Error::Timeout => StorageError::Timeout,
        nor::Error::InvalidRegion | nor::Error::InvalidArgument => StorageError::InvalidArgument,
    }
}

pub struct Mt8196Nor {
    nor: MtkNor,
    size: u64,
    dma: u64,
}

impl Mt8196Nor {
    /// `size` is the boot media size from the coreboot tables.
    ///
    /// # Safety
    /// `base` is the SFLASH controller, Device-mapped and clocked by
    /// coreboot. Only one instance may exist: it owns the static DMA buffer.
    unsafe fn new(base: u64, size: u64) -> Result<Self, StorageError> {
        // SAFETY: the caller's contract.
        let nor = unsafe { MtkNor::new(base as usize) }.map_err(storage_error)?;
        Ok(Self {
            nor,
            size,
            dma: core::ptr::addr_of_mut!(DMA_BUFFER) as u64,
        })
    }

    /// Flash status register 1.
    pub fn status(&mut self) -> Result<u8, StorageError> {
        self.nor.status().map_err(storage_error)
    }

    fn check(&self, offset: u64, len: usize) -> Result<u32, StorageError> {
        offset
            .checked_add(len as u64)
            .filter(|&end| end <= self.size)
            .and_then(|_| u32::try_from(offset).ok())
            .ok_or(StorageError::InvalidArgument)
    }

    fn read_at(&mut self, offset: u64, buffer: &mut [u8]) -> Result<(), StorageError> {
        let addr = self.check(offset, buffer.len())?;
        let mut done = 0usize;
        while done < buffer.len() {
            let pos = addr + done as u32;
            let start = pos & !(nor::DMA_ALIGN - 1);
            let skip = (pos - start) as usize;
            let want = (buffer.len() - done).min(DMA_LEN - skip);
            let len = (skip + want).next_multiple_of(nor::DMA_ALIGN as usize) as u32;
            crabefi::arch::flush_cache_range(self.dma, len as usize);
            if let Err(error) = self.nor.read_dma(start, self.dma as u32, len) {
                log::warn!("MT8196 NOR: DMA read at {start:#x}: {error:?}");
                return Err(storage_error(error));
            }
            crabefi::arch::invalidate_cache_range(self.dma, len as usize);
            // SAFETY: the DMA page holds `len >= skip + want` freshly read bytes.
            let source =
                unsafe { core::slice::from_raw_parts(self.dma as *const u8, len as usize) };
            buffer[done..done + want].copy_from_slice(&source[skip..skip + want]);
            done += want;
        }
        Ok(())
    }

    fn program_at(&mut self, offset: u64, data: &[u8]) -> Result<(), StorageError> {
        let addr = self.check(offset, data.len())?;
        self.nor.program(addr, data).map_err(storage_error)
    }

    fn erase_at(&mut self, offset: u64, size: u64) -> Result<(), StorageError> {
        let addr = self.check(offset, size as usize)?;
        let size = u32::try_from(size).map_err(|_| StorageError::InvalidArgument)?;
        self.nor.erase(addr, size).map_err(storage_error)
    }
}

impl FirmwareStorage for Mt8196Nor {
    fn name(&self) -> &str {
        "MT8196 SPI NOR"
    }

    fn capacity(&self) -> Option<u64> {
        Some(self.size)
    }

    fn enable_writes(&mut self) -> Result<(), StorageError> {
        if self.status()? & 0x7c != 0 {
            return Err(StorageError::WriteProtected);
        }
        Ok(())
    }

    fn read(&mut self, offset: u64, buffer: &mut [u8]) -> Result<(), StorageError> {
        self.read_at(offset, buffer)
    }

    fn write(&mut self, offset: u64, data: &[u8]) -> Result<(), StorageError> {
        self.enable_writes()?;
        self.program_at(offset, data)
    }

    fn erase(&mut self, offset: u64, size: u64) -> Result<(), StorageError> {
        self.enable_writes()?;
        self.erase_at(offset, size)
    }
}

/// The FMAP variable region of the NOR flash.
pub struct VariableRegion {
    nor: Mt8196Nor,
    /// SFLASH controller base, for the runtime handoff.
    sflash: u64,
    base: u64,
    size: u32,
    protected: bool,
}

impl StorageBackend for VariableRegion {
    fn name(&self) -> &str {
        "MT8196 SPI NOR NVRAM"
    }

    fn size(&self) -> u32 {
        self.size
    }

    fn program_granularity(&self) -> u32 {
        1
    }

    fn erase_granularity(&self) -> u32 {
        SECTOR
    }

    fn is_write_protected(&self) -> bool {
        self.protected
    }

    fn read(&mut self, offset: u32, buffer: &mut [u8]) -> Result<(), StorageError> {
        bounded(self.size, offset, buffer.len())?;
        self.nor.read_at(self.base + u64::from(offset), buffer)
    }

    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), StorageError> {
        bounded(self.size, offset, data.len())?;
        self.nor.program_at(self.base + u64::from(offset), data)
    }

    fn erase(&mut self, offset: u32, size: u32) -> Result<(), StorageError> {
        bounded(self.size, offset, size as usize)?;
        self.nor
            .erase_at(self.base + u64::from(offset), u64::from(size))
    }
}

impl VariableRegion {
    /// Runtime in-place persistence of this region (ABI handoff record) and
    /// the device windows it needs after ExitBootServices.
    pub fn runtime_config(
        &self,
    ) -> (
        crabefi::RuntimeNvramConfig,
        &'static [crabefi::RuntimeExternalRange],
    ) {
        const UC_RUNTIME: u64 = 1 | 1 << 63;
        let ranges: &'static [crabefi::RuntimeExternalRange] =
            alloc::boxed::Box::leak(alloc::boxed::Box::new([
                range(self.sflash),
                range(TOPCKGEN),
                range(PERICFG_AO),
            ]));
        const fn range(base: u64) -> crabefi::RuntimeExternalRange {
            crabefi::RuntimeExternalRange {
                physical_base: base,
                byte_len: 0x1000,
                attributes: UC_RUNTIME,
            }
        }
        (
            crabefi::RuntimeNvramConfig {
                mechanism: crabefi::nvram_mechanism::MT8196_NOR,
                reserved: 0,
                mmio_bases: [self.sflash, TOPCKGEN, PERICFG_AO],
                region_offset: self.base as u32,
                region_size: self.size,
            },
            ranges,
        )
    }
}

fn bounded(size: u32, offset: u32, len: usize) -> Result<(), StorageError> {
    u64::from(offset)
        .checked_add(len as u64)
        .filter(|&end| end <= u64::from(size))
        .map(|_| ())
        .ok_or(StorageError::InvalidArgument)
}

/// Locate the FMAP variable region on the boot flash and return its bounded backend.
pub fn variable_region(
    flash_size: u64,
    fmap_offset: Option<u64>,
) -> Option<&'static mut VariableRegion> {
    let Some(base) = super::dt::board().nor else {
        log::info!("MT8196 NOR: none in the board description; variables are volatile");
        return None;
    };
    // SAFETY: the board description's SFLASH controller, Device-mapped and
    // clocked by coreboot; this is the only instance.
    let mut nor = match unsafe { Mt8196Nor::new(base, flash_size) } {
        Ok(nor) => nor,
        Err(error) => {
            log::error!("MT8196 NOR: controller {base:#x}: {error:?}");
            return None;
        }
    };
    log::info!("MT8196 NOR: flash {flash_size:#x}, FMAP at {fmap_offset:x?}");
    let Some(region) = crate::fmap::get_smmstore_from_fmap(&mut nor, fmap_offset) else {
        log::info!("MT8196 NOR: no NVRAM region in FMAP; variables are volatile");
        return None;
    };
    let aligned = region.offset % u64::from(SECTOR) == 0 && region.size % SECTOR == 0;
    if !aligned || region.offset + u64::from(region.size) > flash_size {
        log::warn!(
            "MT8196 NOR: unusable {} region {:#x}+{:#x}",
            region.name,
            region.offset,
            region.size
        );
        return None;
    }
    // Block-protect bits (SR1 bits 2..=6). The protected range depends on the
    // part; treat any set bit as protecting the region rather than guessing.
    let status = match nor.status() {
        Ok(status) => status,
        Err(error) => {
            log::warn!("MT8196 NOR: status read failed ({error:?})");
            return None;
        }
    };
    let protected = status & 0x7c != 0;
    log::info!(
        "MT8196 NOR: {} at {:#x}+{:#x}, SR1 {status:#04x}{}",
        region.name,
        region.offset,
        region.size,
        if protected {
            " (block protection set: read-only)"
        } else {
            ""
        }
    );
    Some(Box::leak(Box::new(VariableRegion {
        nor,
        sflash: base,
        base: region.offset,
        size: region.size,
        protected,
    })))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn region() -> VariableRegion {
        // Register memory for the controller; tests reject access before
        // touching it beyond the constructor's command-enable write.
        let registers = Box::leak(Box::new([0u32; 0x800]));
        let base = registers.as_mut_ptr() as u64;
        VariableRegion {
            // SAFETY: owned host memory standing in for the registers.
            nor: unsafe { Mt8196Nor::new(base, 0x800000) }.unwrap(),
            sflash: base,
            base: 0x7c0000,
            size: 0x40000,
            protected: false,
        }
    }

    #[test]
    fn variable_region_is_bounded() {
        fn assert_send<T: Send>() {}
        assert_send::<VariableRegion>();
        let mut region = region();
        assert_eq!(StorageBackend::size(&region), 0x40000);
        assert_eq!(
            StorageBackend::read(&mut region, 0x40000, &mut [0]),
            Err(StorageError::InvalidArgument)
        );
        assert_eq!(
            StorageBackend::program(&mut region, 0x40000, &[0]),
            Err(StorageError::InvalidArgument)
        );
        assert_eq!(
            StorageBackend::erase(&mut region, 0x40000, SECTOR),
            Err(StorageError::InvalidArgument)
        );
    }
}
