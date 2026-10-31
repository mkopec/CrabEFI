//! In-place runtime persistence of non-volatile variables.
//!
//! After ExitBootServices the boot image's storage backend is gone. Platforms
//! whose variable flash the OS does not drive hand over a [`NvramConfig`];
//! runtime SetVariable then appends the EDK2 authenticated record to the
//! variable FV itself and retires its predecessor, with the same state
//! protocol as the boot-time writer. Compaction needs an erase and stays a
//! boot-time operation: a full store, like any flash error, makes the caller
//! fall back to the retained journal.

use crabefi_runtime_abi::{NvramMechanism, VariableTimestamp};
use zerocopy::IntoBytes;
use zerocopy::byteorder::little_endian::{I16, U16, U32, U64};

use crabefi_hw::mediatek::nor::{self, MtkNor};

use crate::efi;

/// Mechanism and physical MMIO bases accepted from the handoff.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NvramConfig {
    pub mechanism: NvramMechanism,
    pub mmio_bases: [u64; 3],
    pub region_offset: u32,
    pub region_size: u32,
}

impl NvramConfig {
    pub const fn none() -> Self {
        Self {
            mechanism: NvramMechanism::None,
            mmio_bases: [0; 3],
            region_offset: 0,
            region_size: 0,
        }
    }
}

/// One non-volatile commit, as the boot bridge would receive it.
pub struct Commit<'a> {
    pub guid: [u8; 16],
    /// UTF-16 name without its terminator; `NameSize` on flash includes one.
    pub name: &'a [u16],
    pub attributes: u32,
    pub data: &'a [u8],
    pub timestamp: Option<VariableTimestamp>,
    pub delete: bool,
}

/// Persist `commit` through `config`, whose MMIO bases are already
/// translated to currently mapped addresses.
pub fn persist(config: &NvramConfig, commit: &Commit<'_>) -> Result<(), efi::Status> {
    match config.mechanism {
        NvramMechanism::None => Err(efi::Status::UNSUPPORTED),
        NvramMechanism::Mt8196Nor => {
            let mut flash = Mt8196Nor::open(config)?;
            let result = Store::open(&mut flash, config.region_size)
                .and_then(|mut store| store.commit(commit));
            flash.close();
            result
        }
    }
}

/// Bytes of variable data behind the store header, as the boot-time reader
/// accepts them: `Size` counts either the data only (coreboot) or the
/// variable store header too (PI/EDK2), and must fit the region.
fn data_size(vs_size: u32, region_size: u32) -> Option<u32> {
    let capacity = region_size.checked_sub(VARIABLE_DATA_OFFSET)?;
    if vs_size <= capacity {
        return Some(vs_size);
    }
    (vs_size <= region_size.checked_sub(FV_HEADER_LENGTH)?)
        .then(|| vs_size.checked_sub(VS_HEADER_LENGTH))
        .flatten()
        .filter(|&size| size <= capacity)
}

/// Byte access to the variable region (region-relative offsets).
trait Flash {
    fn read(&mut self, offset: u32, buffer: &mut [u8]) -> Result<(), efi::Status>;
    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), efi::Status>;
}

// ---------------------------------------------------------------------------
// EDK2 authenticated variable FV (see crabefi-core varstore/edk2.rs)
// ---------------------------------------------------------------------------

const FV_SIGNATURE: u32 = 0x4856_465f;
const FV_HEADER_LENGTH: u32 = 0x48;
const VS_HEADER_LENGTH: u32 = 0x1c;
const VARIABLE_DATA_OFFSET: u32 = 0x64;
const VARIABLE_STORE_FORMATTED: u8 = 0x5a;
const VARIABLE_STORE_HEALTHY: u8 = 0xfe;
const EFI_AUTH_VARIABLE_GUID: [u8; 16] = [
    0x78, 0x2c, 0xf3, 0xaa, 0x7b, 0x94, 0x9a, 0x43, 0xa1, 0x80, 0x2e, 0x14, 0x4e, 0xc3, 0x77, 0x92,
];
const VARIABLE_DATA: u16 = 0x55aa;
const HEADER_SIZE: u32 = 60;
const VAR_HEADER_VALID_ONLY: u8 = 0x7f;
const VAR_ADDED: u8 = 0x3f;
const VAR_DELETED: u8 = 0xfd;
/// Predecessors retired by one commit (more means an inconsistent store).
const MAX_PREDECESSORS: usize = 4;

#[repr(C, packed)]
#[derive(Clone, Copy, zerocopy::FromBytes, IntoBytes, zerocopy::Immutable)]
struct Header {
    start_id: U16,
    state: u8,
    reserved: u8,
    attributes: U32,
    monotonic_count: U64,
    year: U16,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: u8,
    pad1: u8,
    nanosecond: U32,
    timezone: I16,
    daylight: u8,
    pad2: u8,
    pub_key_index: U32,
    name_size: U32,
    data_size: U32,
    vendor_guid: [u8; 16],
}

const _: () = assert!(core::mem::size_of::<Header>() == HEADER_SIZE as usize);

struct Store<'f, F> {
    flash: &'f mut F,
    end: u32,
}

impl<'f, F: Flash> Store<'f, F> {
    fn open(flash: &'f mut F, region_size: u32) -> Result<Self, efi::Status> {
        let mut header = [0u8; VARIABLE_DATA_OFFSET as usize];
        flash.read(0, &mut header)?;
        let word = |offset: usize| {
            u32::from_le_bytes([
                header[offset],
                header[offset + 1],
                header[offset + 2],
                header[offset + 3],
            ])
        };
        let vs = FV_HEADER_LENGTH as usize;
        if word(0x28) != FV_SIGNATURE
            || header[vs..vs + 16] != EFI_AUTH_VARIABLE_GUID
            || header[vs + 0x14] != VARIABLE_STORE_FORMATTED
            || header[vs + 0x15] != VARIABLE_STORE_HEALTHY
        {
            return Err(efi::Status::VOLUME_CORRUPTED);
        }
        let size = data_size(word(vs + 0x10), region_size).ok_or(efi::Status::VOLUME_CORRUPTED)?;
        Ok(Self {
            flash,
            end: VARIABLE_DATA_OFFSET + size,
        })
    }

    /// Write-ahead walk: the free offset and the active records of `name`.
    fn scan(
        &mut self,
        guid: &[u8; 16],
        name: &[u16],
    ) -> Result<(u32, heapless::Vec<u32, MAX_PREDECESSORS>), efi::Status> {
        let mut predecessors = heapless::Vec::new();
        let mut offset = VARIABLE_DATA_OFFSET;
        while offset + HEADER_SIZE <= self.end {
            let mut raw = [0u8; HEADER_SIZE as usize];
            self.flash.read(offset, &mut raw)?;
            let header: Header = zerocopy::transmute!(raw);
            if header.start_id.get() != VARIABLE_DATA || header.state == 0xff {
                break;
            }
            let (name_size, data_size) = (header.name_size.get(), header.data_size.get());
            if name_size == u32::MAX || data_size == u32::MAX || header.attributes.get() == u32::MAX
            {
                offset += HEADER_SIZE;
                continue;
            }
            let size = HEADER_SIZE
                .checked_add(name_size)
                .and_then(|size| size.checked_add(data_size))
                .and_then(|size| size.checked_next_multiple_of(4))
                .filter(|&size| offset.checked_add(size).is_some_and(|end| end <= self.end))
                .ok_or(efi::Status::VOLUME_CORRUPTED)?;
            if header.state == VAR_ADDED
                && header.vendor_guid == *guid
                && name_size as usize == (name.len() + 1) * 2
                && self.name_matches(offset + HEADER_SIZE, name)?
            {
                predecessors
                    .push(offset + 2)
                    .map_err(|_| efi::Status::VOLUME_CORRUPTED)?;
            }
            offset += size;
        }
        Ok((offset, predecessors))
    }

    fn name_matches(&mut self, offset: u32, name: &[u16]) -> Result<bool, efi::Status> {
        for (index, unit) in name.iter().chain(&[0]).enumerate() {
            let mut bytes = [0u8; 2];
            self.flash.read(offset + 2 * index as u32, &mut bytes)?;
            if u16::from_le_bytes(bytes) != *unit {
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn commit(&mut self, commit: &Commit<'_>) -> Result<(), efi::Status> {
        let (free, predecessors) = self.scan(&commit.guid, commit.name)?;
        // A timestamped deletion keeps an empty record as its replay floor.
        if !(commit.delete && commit.timestamp.is_none()) {
            let data: &[u8] = if commit.delete { &[] } else { commit.data };
            self.append(free, commit, data)?;
        }
        for state in predecessors {
            self.flash.program(state, &[VAR_DELETED])?;
        }
        Ok(())
    }

    fn append(&mut self, offset: u32, commit: &Commit<'_>, data: &[u8]) -> Result<(), efi::Status> {
        if commit.name.is_empty() || commit.name.contains(&0) {
            return Err(efi::Status::INVALID_PARAMETER);
        }
        let name_size = u32::try_from((commit.name.len() + 1) * 2)
            .map_err(|_| efi::Status::INVALID_PARAMETER)?;
        let data_size = u32::try_from(data.len()).map_err(|_| efi::Status::INVALID_PARAMETER)?;
        let size = HEADER_SIZE
            .checked_add(name_size)
            .and_then(|size| size.checked_add(data_size))
            .and_then(|size| size.checked_next_multiple_of(4))
            .ok_or(efi::Status::INVALID_PARAMETER)?;
        if offset.checked_add(size).is_none_or(|end| end > self.end)
            || !self.erased(offset, size)?
        {
            return Err(efi::Status::OUT_OF_RESOURCES);
        }
        let time = commit.timestamp.unwrap_or_default();
        let header = Header {
            start_id: U16::new(VARIABLE_DATA),
            state: 0xff,
            reserved: 0,
            attributes: U32::new(commit.attributes),
            monotonic_count: U64::new(0),
            year: U16::new(time.year),
            month: time.month,
            day: time.day,
            hour: time.hour,
            minute: time.minute,
            second: time.second,
            pad1: 0,
            nanosecond: U32::new(time.nanosecond),
            timezone: I16::new(time.timezone),
            daylight: time.daylight,
            pad2: 0,
            pub_key_index: U32::new(0),
            name_size: U32::new(name_size),
            data_size: U32::new(data_size),
            vendor_guid: commit.guid,
        };
        self.program_verified(offset, header.as_bytes())?;
        self.program_verified(offset + HEADER_SIZE, commit.name.as_bytes())?;
        self.program_verified(offset + HEADER_SIZE + name_size - 2, &[0, 0])?;
        self.program_verified(offset + HEADER_SIZE + name_size, data)?;
        self.flash.program(offset + 2, &[VAR_HEADER_VALID_ONLY])?;
        self.flash.program(offset + 2, &[VAR_ADDED])?;
        let mut state = [0u8];
        self.flash.read(offset + 2, &mut state)?;
        if state[0] != VAR_ADDED {
            return Err(efi::Status::DEVICE_ERROR);
        }
        Ok(())
    }

    fn erased(&mut self, offset: u32, size: u32) -> Result<bool, efi::Status> {
        let mut chunk = [0u8; 64];
        let mut done = 0;
        while done < size {
            let len = (size - done).min(chunk.len() as u32);
            self.flash.read(offset + done, &mut chunk[..len as usize])?;
            if chunk[..len as usize].iter().any(|&byte| byte != 0xff) {
                return Ok(false);
            }
            done += len;
        }
        Ok(true)
    }

    fn program_verified(&mut self, offset: u32, data: &[u8]) -> Result<(), efi::Status> {
        self.flash.program(offset, data)?;
        let mut chunk = [0u8; 64];
        for (index, expected) in data.chunks(chunk.len()).enumerate() {
            let actual = &mut chunk[..expected.len()];
            self.flash.read(offset + (index * 64) as u32, actual)?;
            if actual != expected {
                return Err(efi::Status::DEVICE_ERROR);
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// MT8196 SPI NOR: crabefi-hw's controller driver with PIO reads and page
// programs through the controller's write buffer (no DMA buffer), plus the
// flash clock gates
// ---------------------------------------------------------------------------

/// topckgen CLK_CFG_14 (sflash mux, gate bit 31) and its FENC status.
const CLK_CFG_14: u64 = 0xf0;
const CLK_CFG_14_SET: u64 = 0xf4;
const CLK_CFG_14_CLR: u64 = 0xf8;
const SFLASH_GATE: u32 = 1 << 31;
const CLK_FENC_STATUS_MON_1: u64 = 0x274;
const SFLASH_FENC: u32 = 1 << 4;
/// pericfg_ao CG1: FLASHIF flash/27m/dram/axi/bclk (bits 18..=22).
const PERI_CG1_STA: u64 = 0x14;
const PERI_CG1_SET: u64 = 0x2c;
const PERI_CG1_CLR: u64 = 0x30;
const FLASHIF_GATES: u32 = 0x1f << 18;
/// Poll bound for the clock enable status.
const POLLS: u32 = 2_000_000;

fn read32(address: u64) -> u32 {
    // SAFETY: handoff-validated MMIO inside a retained external range.
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

fn write32(address: u64, value: u32) {
    // SAFETY: as in `read32`.
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

fn status(error: nor::Error) -> efi::Status {
    match error {
        nor::Error::Timeout => efi::Status::TIMEOUT,
        nor::Error::InvalidRegion | nor::Error::InvalidArgument => efi::Status::INVALID_PARAMETER,
    }
}

struct Mt8196Nor {
    nor: Option<MtkNor>,
    topckgen: u64,
    pericfg: u64,
    region_offset: u32,
    region_size: u32,
    /// Gates this call opened and must close again.
    opened_mux: bool,
    opened_gates: u32,
}

impl Mt8196Nor {
    /// Ungate the flash clocks (the OS may have gated them as unused) and
    /// enable the controller's command interface.
    fn open(config: &NvramConfig) -> Result<Self, efi::Status> {
        let [nor, topckgen, pericfg] = config.mmio_bases;
        let mut flash = Self {
            nor: None,
            topckgen,
            pericfg,
            region_offset: config.region_offset,
            region_size: config.region_size,
            opened_mux: false,
            opened_gates: 0,
        };
        if read32(topckgen + CLK_CFG_14) & SFLASH_GATE != 0 {
            write32(topckgen + CLK_CFG_14_CLR, SFLASH_GATE);
            flash.opened_mux = true;
            if !(0..POLLS).any(|_| read32(topckgen + CLK_FENC_STATUS_MON_1) & SFLASH_FENC != 0) {
                flash.close();
                return Err(efi::Status::DEVICE_ERROR);
            }
        }
        flash.opened_gates = read32(pericfg + PERI_CG1_STA) & FLASHIF_GATES;
        if flash.opened_gates != 0 {
            write32(pericfg + PERI_CG1_CLR, flash.opened_gates);
        }
        // SAFETY: handoff-validated controller MMIO, clocked above; runtime
        // services are serialized, so this call is its only user.
        match unsafe { MtkNor::new(nor as usize) } {
            Ok(mut controller) => {
                controller.use_3byte_addresses();
                flash.nor = Some(controller);
                Ok(flash)
            }
            Err(error) => {
                flash.close();
                Err(status(error))
            }
        }
    }

    fn close(&mut self) {
        if self.opened_gates != 0 {
            write32(self.pericfg + PERI_CG1_SET, self.opened_gates);
        }
        if self.opened_mux {
            write32(self.topckgen + CLK_CFG_14_SET, SFLASH_GATE);
        }
    }

    /// The controller and flash address for `len` bytes at region offset
    /// `offset`; no access leaves the region.
    fn controller(&mut self, offset: u32, len: usize) -> Result<(&mut MtkNor, u32), efi::Status> {
        if !in_region(self.region_size, offset, len) {
            return Err(efi::Status::INVALID_PARAMETER);
        }
        let address = self
            .region_offset
            .checked_add(offset)
            .ok_or(efi::Status::INVALID_PARAMETER)?;
        let nor = self.nor.as_mut().ok_or(efi::Status::DEVICE_ERROR)?;
        Ok((nor, address))
    }
}

fn in_region(region_size: u32, offset: u32, len: usize) -> bool {
    u64::from(offset)
        .checked_add(len as u64)
        .is_some_and(|end| end <= u64::from(region_size))
}

impl Flash for Mt8196Nor {
    fn read(&mut self, offset: u32, buffer: &mut [u8]) -> Result<(), efi::Status> {
        let (nor, address) = self.controller(offset, buffer.len())?;
        nor.read_pio(address, buffer).map_err(status)
    }

    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), efi::Status> {
        let (nor, address) = self.controller(offset, data.len())?;
        nor.program(address, data).map_err(status)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// NOR semantics: erased 0xff, programming only clears bits.
    struct Memory(std::vec::Vec<u8>);

    impl Flash for Memory {
        fn read(&mut self, offset: u32, buffer: &mut [u8]) -> Result<(), efi::Status> {
            let start = offset as usize;
            buffer.copy_from_slice(&self.0[start..start + buffer.len()]);
            Ok(())
        }

        fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), efi::Status> {
            for (byte, value) in self.0[offset as usize..].iter_mut().zip(data) {
                *byte &= value;
            }
            Ok(())
        }
    }

    fn formatted(size: usize) -> Memory {
        let mut bytes = std::vec![0xff; size];
        bytes[..0x64].fill(0);
        bytes[0x28..0x2c].copy_from_slice(&FV_SIGNATURE.to_le_bytes());
        bytes[0x48..0x58].copy_from_slice(&EFI_AUTH_VARIABLE_GUID);
        bytes[0x58..0x5c].copy_from_slice(&(size as u32 - 0x64).to_le_bytes());
        bytes[0x5c] = VARIABLE_STORE_FORMATTED;
        bytes[0x5d] = VARIABLE_STORE_HEALTHY;
        Memory(bytes)
    }

    const NAME: [u16; 3] = [b'F' as u16, b'o' as u16, b'o' as u16];

    fn commit<'a>(data: &'a [u8], delete: bool) -> Commit<'a> {
        Commit {
            guid: [7; 16],
            name: &NAME,
            attributes: 7,
            data,
            timestamp: None,
            delete,
        }
    }

    #[test]
    fn update_appends_and_retires_predecessor_then_delete_retires_all() {
        let mut flash = formatted(4096);
        Store::open(&mut flash, 4096)
            .unwrap()
            .commit(&commit(b"one", false))
            .unwrap();
        Store::open(&mut flash, 4096)
            .unwrap()
            .commit(&commit(b"two!", false))
            .unwrap();
        let first = 0x64usize;
        let second = first + (60 + 8 + 3usize).next_multiple_of(4);
        assert_eq!(flash.0[first + 2], VAR_ADDED & VAR_DELETED);
        assert_eq!(flash.0[second + 2], VAR_ADDED);
        assert_eq!(&flash.0[second + 0x24..second + 0x28], &8u32.to_le_bytes());
        assert_eq!(&flash.0[second + 66..second + 68], &[0, 0]);
        assert_eq!(&flash.0[second + 68..second + 72], b"two!");
        let (free, active) = Store::open(&mut flash, 4096)
            .unwrap()
            .scan(&[7; 16], &NAME)
            .unwrap();
        assert_eq!(
            (free as usize, active.as_slice()),
            (second + 72, &[second as u32 + 2][..])
        );
        Store::open(&mut flash, 4096)
            .unwrap()
            .commit(&commit(&[], true))
            .unwrap();
        assert_eq!(flash.0[second + 2], VAR_ADDED & VAR_DELETED);
        assert_eq!(flash.0[second + 72], 0xff, "plain deletion appends nothing");
    }

    #[test]
    fn full_store_and_foreign_formats_are_rejected() {
        let mut flash = formatted(0x64 + 80);
        let big = [0u8; 64];
        assert_eq!(
            Store::open(&mut flash, 0x64 + 80)
                .unwrap()
                .commit(&commit(&big, false)),
            Err(efi::Status::OUT_OF_RESOURCES)
        );
        flash.0[0x48] = 0;
        assert!(Store::open(&mut flash, 0x64 + 80).is_err());
    }

    #[test]
    fn unhealthy_or_oversized_stores_are_rejected() {
        let mut flash = formatted(4096);
        flash.0[0x48 + 0x15] = 0;
        assert!(Store::open(&mut flash, 4096).is_err(), "store state");
        let mut flash = formatted(4096);
        flash.0[0x48 + 0x14] = 0;
        assert!(Store::open(&mut flash, 4096).is_err(), "store format");
        let mut flash = formatted(4096);
        flash.0[0x58..0x5c].copy_from_slice(&(4096u32 - 0x48 + 4).to_le_bytes());
        assert!(Store::open(&mut flash, 4096).is_err(), "size beyond region");
    }

    #[test]
    fn store_size_follows_both_conventions() {
        // coreboot: data only; PI/EDK2: including the 0x1c byte store header.
        assert_eq!(data_size(4096 - 0x64, 4096), Some(4096 - 0x64));
        assert_eq!(data_size(4096 - 0x48, 4096), Some(4096 - 0x64));
        assert_eq!(data_size(4096 - 0x48 + 1, 4096), None);
        assert_eq!(data_size(0, 0x10), None);
    }

    #[test]
    fn accesses_stay_in_the_region() {
        assert!(in_region(4096, 4095, 1));
        assert!(in_region(4096, 4096, 0));
        assert!(!in_region(4096, 4095, 2));
        assert!(!in_region(4096, u32::MAX, 2));
    }
}
