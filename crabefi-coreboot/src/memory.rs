//! Memory map handling for coreboot
//!
//! This module defines the memory region types and provides utilities
//! for working with the memory map from coreboot.

/// Memory region types from coreboot
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum MemoryType {
    /// Usable RAM
    Ram = 1,
    /// Reserved memory
    Reserved = 2,
    /// ACPI reclaimable memory
    AcpiReclaimable = 3,
    /// ACPI NVS (Non-Volatile Storage)
    AcpiNvs = 4,
    /// Unusable memory
    Unusable = 5,
    /// Coreboot tables
    Table = 16,
}

impl TryFrom<u32> for MemoryType {
    type Error = u32;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(MemoryType::Ram),
            2 => Ok(MemoryType::Reserved),
            3 => Ok(MemoryType::AcpiReclaimable),
            4 => Ok(MemoryType::AcpiNvs),
            5 => Ok(MemoryType::Unusable),
            16 => Ok(MemoryType::Table),
            other => Err(other),
        }
    }
}

/// A memory region descriptor
#[derive(Debug, Clone, Copy)]
pub struct MemoryRegion {
    /// Starting physical address
    pub start: u64,
    /// Size in bytes
    pub size: u64,
    /// Type of memory
    pub region_type: MemoryType,
}

/// Whether `[start, start + size)` overlaps RAM in `regions`.
pub fn overlaps_ram(regions: &[MemoryRegion], start: u64, size: u64) -> bool {
    let end = start.saturating_add(size);
    regions.iter().any(|region| {
        region.region_type == MemoryType::Ram
            && region.start < end
            && start < region.start.saturating_add(region.size)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ram_overlap_ignores_reserved_ranges() {
        let regions = [
            MemoryRegion {
                start: 0x8000_0000,
                size: 0x7ffd_9000,
                region_type: MemoryType::Ram,
            },
            MemoryRegion {
                start: 0xfffd_9000,
                size: 0x2_7000,
                region_type: MemoryType::Reserved,
            },
        ];
        assert!(overlaps_ram(&regions, 0xff80_0000, 0x80_0000));
        assert!(!overlaps_ram(&regions, 0xfffe_0000, 0x2_0000));
        assert!(!overlaps_ram(&regions, 0x7f80_0000, 0x80_0000));
        assert!(!overlaps_ram(&[], 0xff80_0000, 0x80_0000));
    }
}
