//! AArch64 BLS boot through the Linux EFI stub, with an external DTB and initrd.
//! Disk-supplied auxiliary data is deliberately disabled under Secure Boot.

#![cfg_attr(not(target_arch = "aarch64"), allow(dead_code))]

use crate::{
    boot,
    drivers::storage,
    efi::{self, allocator, boot_services, system_table},
    fs, menu,
};
use alloc::{boxed::Box, vec::Vec};
use core::{ffi::c_void, ptr};
use r_efi::{
    efi::{Boolean, Guid, Handle, Status},
    protocols::{device_path, load_file2},
};

const MAX_INITRD: usize = 512 * 1024 * 1024;
const MAX_DTB: usize = 16 * 1024 * 1024;
const INITRD_GUID: Guid = Guid::from_fields(
    0x5568e427,
    0x68fc,
    0x4f3d,
    0xac,
    0x74,
    &[0xca, 0x55, 0x52, 0x31, 0xcc, 0x68],
);

/// Page-backed buffer: the firmware heap is only a few MiB, initrds are tens.
struct Buffer {
    ptr: *mut u8,
    len: usize,
    /// Owned EFI pages, or 0 for a borrowed test buffer.
    pages: u64,
}

impl Buffer {
    fn allocate(len: usize) -> Result<Self, Status> {
        let pages = (len.max(1) as u64).div_ceil(4096);
        let mut address = 0;
        let status = allocator::allocate_pages(
            allocator::AllocateType::AllocateAnyPages,
            allocator::MemoryType::LoaderData,
            pages,
            &mut address,
        );
        if status != Status::SUCCESS {
            log::error!("Linux EFI: cannot allocate {len} bytes: {status:?}");
            return Err(status);
        }
        Ok(Self {
            ptr: address as *mut u8,
            len,
            pages,
        })
    }

    #[cfg(test)]
    fn borrowed(data: &mut [u8]) -> Self {
        Self {
            ptr: data.as_mut_ptr(),
            len: data.len(),
            pages: 0,
        }
    }

    fn as_slice(&self) -> &[u8] {
        // SAFETY: ptr covers len bytes for the buffer's lifetime.
        unsafe { core::slice::from_raw_parts(self.ptr, self.len) }
    }

    fn as_mut_slice(&mut self) -> &mut [u8] {
        // SAFETY: as above, exclusively owned.
        unsafe { core::slice::from_raw_parts_mut(self.ptr, self.len) }
    }
}

impl Drop for Buffer {
    fn drop(&mut self) {
        if self.pages != 0 {
            let _ = allocator::free_pages(self.ptr as u64, self.pages);
        }
    }
}

fn file_size(entry: &menu::BootEntry, path: &str) -> Result<usize, Status> {
    let path = fs::linux_path_to_fat(path).map_err(|_| Status::INVALID_PARAMETER)?;
    storage::with_disk(entry.storage, |disk| {
        let mut fat = fs::fat::FatFilesystem::new(disk, entry.partition.first_lba)
            .map_err(|_| Status::VOLUME_CORRUPTED)?;
        fat.file_size(&path).map(|size| size as usize).map_err(|_| {
            log::error!("Linux EFI: {path} not found");
            Status::NOT_FOUND
        })
    })
    .map_err(|_| Status::DEVICE_ERROR)?
}

fn read_into(entry: &menu::BootEntry, path: &str, buffer: &mut [u8]) -> Result<(), Status> {
    let path = fs::linux_path_to_fat(path).map_err(|_| Status::INVALID_PARAMETER)?;
    storage::with_disk(entry.storage, |disk| {
        let mut fat = fs::fat::FatFilesystem::new(disk, entry.partition.first_lba)
            .map_err(|_| Status::VOLUME_CORRUPTED)?;
        let read = fat
            .read_file_all(&path, buffer)
            .map_err(|_| Status::DEVICE_ERROR)?;
        if read != buffer.len() {
            return Err(Status::DEVICE_ERROR);
        }
        log::info!("Linux EFI: loaded {path} ({read} bytes)");
        Ok(())
    })
    .map_err(|_| Status::DEVICE_ERROR)?
}

fn read_file(entry: &menu::BootEntry, path: &str, limit: usize) -> Result<Buffer, Status> {
    let length = file_size(entry, path)?;
    if length == 0 || length > limit {
        return Err(Status::BAD_BUFFER_SIZE);
    }
    let mut buffer = Buffer::allocate(length)?;
    read_into(entry, path, buffer.as_mut_slice())?;
    Ok(buffer)
}

/// Read all initrds, in order, into one page-backed buffer.
fn read_initrds(
    entry: &menu::BootEntry,
    paths: &crate::bls::Initrds,
) -> Result<Option<Buffer>, Status> {
    let mut sizes = heapless::Vec::<usize, { crate::bls::MAX_INITRDS }>::new();
    let mut total = 0usize;
    for path in paths {
        let size = file_size(entry, path)?;
        total = total.checked_add(size).ok_or(Status::BAD_BUFFER_SIZE)?;
        let _ = sizes.push(size);
    }
    if total == 0 {
        return Ok(None);
    }
    if total > MAX_INITRD {
        return Err(Status::BAD_BUFFER_SIZE);
    }
    let mut buffer = Buffer::allocate(total)?;
    let mut offset = 0;
    for (path, size) in paths.iter().zip(sizes) {
        read_into(
            entry,
            path,
            &mut buffer.as_mut_slice()[offset..offset + size],
        )?;
        offset += size;
    }
    Ok(Some(buffer))
}

fn current_dtb() -> *mut c_void {
    let table = system_table::get_system_table_efi();
    // SAFETY: the EFI environment is initialized before the boot manager runs.
    unsafe {
        if table.is_null() {
            return ptr::null_mut();
        }
        for index in 0..(*table).number_of_table_entries {
            let entry = &*(*table).configuration_table.add(index);
            if entry.vendor_guid == system_table::EFI_DTB_TABLE_GUID {
                return entry.vendor_table;
            }
        }
    }
    ptr::null_mut()
}

struct DtbTable {
    address: u64,
    pages: u64,
    previous: *mut c_void,
}

impl DtbTable {
    fn install(bytes: &[u8]) -> Result<Self, Status> {
        let length = crate::fdt_tree::validate(bytes).ok_or(Status::COMPROMISED_DATA)?;
        let pages = (length as u64).div_ceil(4096);
        let mut address = 0;
        let status = allocator::allocate_pages(
            allocator::AllocateType::AllocateAnyPages,
            allocator::MemoryType::LoaderData,
            pages,
            &mut address,
        );
        if status != Status::SUCCESS {
            return Err(status);
        }
        // SAFETY: the allocator returned exclusive, page-aligned storage.
        unsafe {
            ptr::copy_nonoverlapping(bytes.as_ptr(), address as *mut u8, length);
        }
        let previous = current_dtb();
        let status = system_table::install_configuration_table(
            &system_table::EFI_DTB_TABLE_GUID,
            address as *mut c_void,
        );
        if status != Status::SUCCESS {
            let _ = allocator::free_pages(address, pages);
            return Err(status);
        }
        log::info!("Linux EFI: DTB installed at {address:#x}, {length} bytes");
        Ok(Self {
            address,
            pages,
            previous,
        })
    }
}

impl Drop for DtbTable {
    fn drop(&mut self) {
        // Never free a table still published in the configuration array.
        let status = system_table::install_configuration_table(
            &system_table::EFI_DTB_TABLE_GUID,
            self.previous,
        );
        if status == Status::SUCCESS {
            let _ = allocator::free_pages(self.address, self.pages);
        } else {
            log::error!("Linux EFI: cannot restore DTB table ({status:?}); retaining its pages");
        }
    }
}

/// Protocol is the first field, so the callback can recover its owner.
#[repr(C)]
struct InitrdProvider {
    protocol: load_file2::Protocol,
    path: [u8; 24],
    data: Buffer,
    handle: Handle,
    path_installed: bool,
    protocol_installed: bool,
}

unsafe extern "efiapi" fn load_initrd(
    this: *mut load_file2::Protocol,
    path: *mut device_path::Protocol,
    boot_policy: Boolean,
    buffer_size: *mut usize,
    buffer: *mut c_void,
) -> Status {
    if this.is_null() || path.is_null() || buffer_size.is_null() {
        return Status::INVALID_PARAMETER;
    }
    if bool::from(boot_policy) {
        return Status::UNSUPPORTED;
    }
    // SAFETY: UEFI callers supply a valid device-path node and size pointer;
    // this points to the first field of our live, boxed provider.
    unsafe {
        if ptr::read_unaligned(path.cast::<[u8; 4]>()) != [0x7f, 0xff, 4, 0] {
            return Status::NOT_FOUND;
        }
        let provider = &*this.cast::<InitrdProvider>();
        let length = provider.data.len;
        if buffer.is_null() || *buffer_size < length {
            *buffer_size = length;
            return Status::BUFFER_TOO_SMALL;
        }
        ptr::copy_nonoverlapping(provider.data.ptr, buffer.cast::<u8>(), length);
        *buffer_size = length;
    }
    Status::SUCCESS
}

impl InitrdProvider {
    fn new(data: Buffer) -> Box<Self> {
        let mut path = [0u8; 24];
        path[..4].copy_from_slice(&[4, 3, 20, 0]); // MEDIA_VENDOR_DP
        path[4..20].copy_from_slice(INITRD_GUID.as_bytes());
        path[20..].copy_from_slice(&[0x7f, 0xff, 4, 0]);
        Box::new(Self {
            protocol: load_file2::Protocol {
                load_file: load_initrd,
            },
            path,
            data,
            handle: ptr::null_mut(),
            path_installed: false,
            protocol_installed: false,
        })
    }

    fn install(data: Buffer) -> Result<Box<Self>, Status> {
        let mut provider = Self::new(data);
        provider.handle = boot_services::create_handle().ok_or(Status::OUT_OF_RESOURCES)?;
        let status = boot_services::install_protocol(
            provider.handle,
            &device_path::PROTOCOL_GUID,
            provider.path.as_mut_ptr().cast(),
        );
        if status != Status::SUCCESS {
            return Err(status);
        }
        provider.path_installed = true;
        let status = boot_services::install_protocol(
            provider.handle,
            &load_file2::PROTOCOL_GUID,
            (&raw mut provider.protocol).cast(),
        );
        if status != Status::SUCCESS {
            Self::release(provider);
            return Err(status);
        }
        provider.protocol_installed = true;
        log::info!(
            "Linux EFI: LoadFile2 initrd published ({} bytes)",
            provider.data.len
        );
        Ok(provider)
    }

    fn release(mut provider: Box<Self>) {
        // SAFETY: Boot Services is initialized and both published interfaces
        // point into this allocation, retained until uninstallation succeeds.
        let bs = unsafe { &*boot_services::get_boot_services() };
        if provider.protocol_installed {
            let mut guid = load_file2::PROTOCOL_GUID;
            let status = unsafe {
                (bs.uninstall_protocol_interface)(
                    provider.handle,
                    &mut guid,
                    (&raw mut provider.protocol).cast(),
                )
            };
            if status == Status::SUCCESS {
                provider.protocol_installed = false;
            }
        }
        if provider.path_installed {
            let mut guid = device_path::PROTOCOL_GUID;
            let status = unsafe {
                (bs.uninstall_protocol_interface)(
                    provider.handle,
                    &mut guid,
                    provider.path.as_mut_ptr().cast(),
                )
            };
            if status == Status::SUCCESS {
                provider.path_installed = false;
            }
        }
        if provider.path_installed || provider.protocol_installed {
            log::error!("Linux EFI: initrd protocol still referenced; retaining its storage");
            let _ = Box::leak(provider);
        }
    }
}

pub(crate) fn boot_entry(
    entry: &menu::BootEntry,
    kernel: &str,
    initrds: &crate::bls::Initrds,
    cmdline: &str,
    dtb_path: Option<&str>,
) -> Result<(), Status> {
    if efi::auth::is_secure_boot_enabled() {
        log::error!("External Linux DTB/initrd boot is disabled under Secure Boot");
        return Err(Status::SECURITY_VIOLATION);
    }
    if cmdline.contains('\0') {
        return Err(Status::INVALID_PARAMETER);
    }
    let kernel = fs::linux_path_to_fat(kernel).map_err(|_| Status::INVALID_PARAMETER)?;
    let options: Vec<u16> = cmdline.encode_utf16().chain(core::iter::once(0)).collect();

    // Read and validate all auxiliary files before publishing any interfaces.
    let tree = dtb_path
        .map(|path| read_file(entry, path, MAX_DTB))
        .transpose()?;
    if let Some(ref bytes) = tree {
        crate::fdt_tree::validate(bytes.as_slice()).ok_or(Status::COMPROMISED_DATA)?;
    } else if current_dtb().is_null() {
        log::error!("Linux EFI: no platform DTB; specify devicetree in the BLS entry");
        return Err(Status::NOT_FOUND);
    }
    let data = read_initrds(entry, initrds)?;
    let info =
        storage::with_disk(entry.storage, |disk| disk.info()).map_err(|_| Status::DEVICE_ERROR)?;
    let path_info = boot::device_path_info_from_entry(entry);
    boot::install_block_io_protocols(entry.storage, info, &path_info);
    // The entry's devicetree replaces the platform's: add the firmware's nodes.
    let fixed = tree
        .as_ref()
        .and_then(|bytes| crate::efi::protocols::dt_fixup::apply_platform_fixups(bytes.as_slice()));
    let _dtb = match (&fixed, &tree) {
        (Some(fixed), _) => Some(DtbTable::install(fixed)?),
        (None, Some(bytes)) => Some(DtbTable::install(bytes.as_slice())?),
        (None, None) => None,
    };
    let initrd = data.map(InitrdProvider::install).transpose()?;
    log::info!("Linux EFI: starting {kernel}, command line: {cmdline}");
    let result = boot::try_boot_file_from_esp(
        &entry.partition,
        entry.partition_num,
        &path_info,
        entry.storage,
        info.block_size,
        &kernel,
        &options,
    );
    if let Some(provider) = initrd {
        InitrdProvider::release(provider);
    }
    if result {
        Ok(())
    } else {
        Err(Status::LOAD_ERROR)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn initrd_protocol_queries_size_and_copies_concatenated_bytes() {
        let mut storage = vec![1, 2, 3, 4, 5];
        let mut provider = InitrdProvider::new(Buffer::borrowed(&mut storage));
        let mut end = [0x7fu8, 0xff, 4, 0];
        let path = end.as_mut_ptr().cast();
        let mut size = 0;
        let protocol = &raw mut provider.protocol;
        assert_eq!(
            unsafe { load_initrd(protocol, path, Boolean::FALSE, &mut size, ptr::null_mut()) },
            Status::BUFFER_TOO_SMALL
        );
        assert_eq!(size, 5);
        let mut buffer = [0u8; 8];
        size = 8;
        assert_eq!(
            unsafe {
                load_initrd(
                    protocol,
                    path,
                    Boolean::FALSE,
                    &mut size,
                    buffer.as_mut_ptr().cast(),
                )
            },
            Status::SUCCESS
        );
        assert_eq!(size, 5);
        assert_eq!(buffer, [1, 2, 3, 4, 5, 0, 0, 0]);
    }

    #[test]
    fn initrd_protocol_rejects_invalid_requests() {
        let mut storage = vec![1];
        let mut provider = InitrdProvider::new(Buffer::borrowed(&mut storage));
        let mut end = [0x7fu8, 0xff, 4, 0];
        let protocol = &raw mut provider.protocol;
        let path = end.as_mut_ptr().cast();
        let mut size = 0;
        assert_eq!(
            unsafe { load_initrd(protocol, path, Boolean::TRUE, &mut size, ptr::null_mut()) },
            Status::UNSUPPORTED
        );
        assert_eq!(
            unsafe {
                load_initrd(
                    protocol,
                    path,
                    Boolean::FALSE,
                    ptr::null_mut(),
                    ptr::null_mut(),
                )
            },
            Status::INVALID_PARAMETER
        );
        end[0] = 4;
        let path = end.as_mut_ptr().cast();
        assert_eq!(
            unsafe { load_initrd(protocol, path, Boolean::FALSE, &mut size, ptr::null_mut()) },
            Status::NOT_FOUND
        );
    }

    #[test]
    fn initrd_vendor_path_matches_linux_guid() {
        let provider = InitrdProvider::new(Buffer::borrowed(&mut []));
        assert_eq!(&provider.path[..4], &[4, 3, 20, 0]);
        assert_eq!(&provider.path[4..20], INITRD_GUID.as_bytes());
        assert_eq!(&provider.path[20..], &[0x7f, 0xff, 4, 0]);
    }
}
