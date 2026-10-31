//! EFI Devicetree Fixup Protocol
//!
//! Lets an OS loader that supplies its own devicetree (systemd-stub with a
//! UKI `.dtb`, GRUB `devicetree`) have the firmware apply its fixups first,
//! so nodes only the firmware knows reach the OS. The platform contributes
//! its fixups through [`PlatformHooks::fixup_devicetree`].
//!
//! Reference: EBBR 2.1, Section 2.3.4; U-Boot `efi_dt_fixup_protocol`.
//!
//! [`PlatformHooks::fixup_devicetree`]: crate::platform::PlatformHooks::fixup_devicetree

use alloc::vec::Vec;
use core::ffi::c_void;

use r_efi::efi::{Guid, Status};

use crate::efi::allocator::{self, AllocateType, MemoryType};
use crate::efi::system_table;
use crate::efi::utils::allocate_protocol_with_log;
use crate::fdt_tree::Tree;

/// {E617D64C-FE08-46DA-F4DC-BBD5870C7300}
pub const DT_FIXUP_PROTOCOL_GUID: Guid = Guid::from_fields(
    0xe617_d64c,
    0xfe08,
    0x46da,
    0xf4,
    0xdc,
    &[0xbb, 0xd5, 0x87, 0x0c, 0x73, 0x00],
);

const REVISION: u64 = 0x0001_0000;

/// Apply the firmware's fixups to the devicetree.
pub const APPLY_FIXUPS: u32 = 0x1;
/// Reserve the devicetree's reserved memory in the UEFI memory map.
pub const RESERVE_MEMORY: u32 = 0x2;
/// Install the devicetree as the EFI DTB configuration table.
pub const INSTALL_TABLE: u32 = 0x4;
const ALL_FLAGS: u32 = APPLY_FIXUPS | RESERVE_MEMORY | INSTALL_TABLE;

#[repr(C)]
pub struct Protocol {
    pub revision: u64,
    pub fixup: extern "efiapi" fn(
        this: *mut Protocol,
        fdt: *mut c_void,
        buffer_size: *mut usize,
        flags: u32,
    ) -> Status,
}

/// The devicetree with the platform's fixups, or `None` when they change
/// nothing (the blob is then left exactly as it is).
fn fixed_up(tree: &Tree) -> Option<Vec<u8>> {
    let hooks = crate::handoff::callbacks().hooks?;
    let mut fixed = tree.clone();
    hooks.fixup_devicetree(&mut fixed);
    (fixed != *tree).then(|| fixed.to_bytes())
}

/// Apply the platform fixups to a devicetree the firmware itself installs
/// (BLS `devicetree` entries, booted by `efi_linux`).
#[cfg(any(target_arch = "aarch64", test))]
pub(crate) fn apply_platform_fixups(blob: &[u8]) -> Option<Vec<u8>> {
    fixed_up(&Tree::parse(blob)?)
}

fn reserve(address: u64, size: u64, no_map: bool) {
    let start = address & !0xfff;
    let Some(end) = address
        .checked_add(size)
        .and_then(|end| end.checked_next_multiple_of(0x1000))
    else {
        return;
    };
    if size == 0 {
        return;
    }
    // Only free memory is claimed; regions the firmware already owns or that
    // are not RAM stay as they are, as with U-Boot's carve-out.
    let memory_type = if no_map {
        MemoryType::ReservedMemoryType
    } else {
        MemoryType::BootServicesData
    };
    let mut base = start;
    let status = allocator::allocate_pages(
        AllocateType::AllocateAddress,
        memory_type,
        (end - start) / 0x1000,
        &mut base,
    );
    log::debug!("DT fixup: reserve {start:#x}..{end:#x} no-map={no_map}: {status:?}");
}

/// Reserve `/memreserve/` entries and `/reserved-memory` regions.
fn reserve_memory(tree: &Tree) {
    for &(address, size) in &tree.reservations {
        reserve(address, size, true);
    }
    let Some(reserved) = tree.node("/reserved-memory") else {
        return;
    };
    for region in &reserved.children {
        if region
            .property("status")
            .is_some_and(|status| status != &b"okay\0"[..] && status != &b"ok\0"[..])
        {
            continue;
        }
        let no_map = region.property("no-map").is_some();
        for (address, size) in region
            .property("reg")
            .and_then(|reg| reserved.decode_reg(reg))
            .into_iter()
            .flatten()
        {
            reserve(address, size, no_map);
        }
    }
}

extern "efiapi" fn fixup(
    this: *mut Protocol,
    fdt: *mut c_void,
    buffer_size: *mut usize,
    flags: u32,
) -> Status {
    if this.is_null()
        || fdt.is_null()
        || buffer_size.is_null()
        || flags == 0
        || flags & !ALL_FLAGS != 0
    {
        return Status::INVALID_PARAMETER;
    }
    // SAFETY: the caller passes a buffer of `*buffer_size` bytes at `fdt`.
    let capacity = unsafe { *buffer_size };
    let blob = unsafe { core::slice::from_raw_parts(fdt as *const u8, capacity) };
    let Some(tree) = Tree::parse(blob) else {
        return Status::INVALID_PARAMETER;
    };

    let mut tree_for_reserve = None;
    if flags & APPLY_FIXUPS != 0
        && let Some(fixed) = fixed_up(&tree)
    {
        if fixed.len() > capacity {
            // SAFETY: checked non-null above.
            unsafe { *buffer_size = fixed.len() };
            return Status::BUFFER_TOO_SMALL;
        }
        // SAFETY: `fixed` fits the caller's buffer.
        unsafe { core::ptr::copy_nonoverlapping(fixed.as_ptr(), fdt as *mut u8, fixed.len()) };
        log::info!("DT fixup: applied platform fixups ({} bytes)", fixed.len());
        tree_for_reserve = Tree::parse(&fixed);
    }
    if flags & RESERVE_MEMORY != 0 {
        reserve_memory(tree_for_reserve.as_ref().unwrap_or(&tree));
    }
    if flags & INSTALL_TABLE != 0 {
        let status =
            system_table::install_configuration_table(&system_table::EFI_DTB_TABLE_GUID, fdt);
        if status != Status::SUCCESS {
            return status;
        }
    }
    Status::SUCCESS
}

/// Create the protocol instance.
pub fn create_protocol() -> *mut Protocol {
    allocate_protocol_with_log::<Protocol>("DtFixupProtocol", |protocol| {
        protocol.revision = REVISION;
        protocol.fixup = fixup;
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fdt_tree::Node;
    use alloc::vec;

    /// Adds `/fixed` unless the tree already has it.
    struct AddNode;

    impl crate::platform::PlatformHooks for AddNode {
        fn fixup_devicetree(&self, tree: &mut Tree) {
            if tree.node("/fixed").is_none() {
                let mut node = Node::new("fixed");
                node.set_property("compatible", b"crabefi,test\0");
                tree.root.children.push(node);
            }
        }
    }

    static HOOKS: AddNode = AddNode;

    /// The callbacks are process-wide: install the same hooks exactly once.
    fn install_hooks() {
        static ONCE: std::sync::Once = std::sync::Once::new();
        ONCE.call_once(|| {
            crate::handoff::set_callbacks(crate::handoff::Callbacks {
                hooks: Some(&HOOKS),
                ..crate::handoff::callbacks()
            })
        });
    }

    /// A devicetree with an empty root and, optionally, a `/fixed` node.
    fn blob(with_fixed: bool) -> Vec<u8> {
        let word = |out: &mut Vec<u8>, value: u32| out.extend(value.to_be_bytes());
        let mut structure = Vec::new();
        word(&mut structure, 1); // FDT_BEGIN_NODE ""
        word(&mut structure, 0);
        if with_fixed {
            word(&mut structure, 1); // FDT_BEGIN_NODE "fixed"
            structure.extend(b"fixed\0\0\0");
            word(&mut structure, 2); // FDT_END_NODE
        }
        word(&mut structure, 2); // FDT_END_NODE
        word(&mut structure, 9); // FDT_END
        // Header (40 bytes), empty reservation map (16), structure, no strings.
        let structure_offset = 56u32;
        let total = structure_offset + structure.len() as u32;
        let mut out = Vec::new();
        for value in [
            0xd00d_feed,
            total,
            structure_offset,
            total, // strings
            40,    // reservation map
            17,
            16,
            0,
            0,
            structure.len() as u32,
        ] {
            word(&mut out, value);
        }
        out.resize(56, 0); // header padding and the empty reservation map
        out.extend(structure);
        out
    }

    fn call(buffer: &mut [u8], size: &mut usize, flags: u32) -> Status {
        let mut protocol = Protocol {
            revision: REVISION,
            fixup,
        };
        fixup(&mut protocol, buffer.as_mut_ptr().cast(), size, flags)
    }

    #[test]
    fn rejects_bad_arguments_and_blobs() {
        let mut tree = blob(false);
        let mut size = tree.len();
        for flags in [0, ALL_FLAGS + 1, 0x8] {
            assert_eq!(call(&mut tree, &mut size, flags), Status::INVALID_PARAMETER);
        }
        let mut protocol = Protocol {
            revision: REVISION,
            fixup,
        };
        assert_eq!(
            fixup(
                &mut protocol,
                core::ptr::null_mut(),
                &mut size,
                APPLY_FIXUPS
            ),
            Status::INVALID_PARAMETER
        );
        let mut garbage = vec![0u8; 64];
        let mut size = garbage.len();
        assert_eq!(
            call(&mut garbage, &mut size, APPLY_FIXUPS),
            Status::INVALID_PARAMETER
        );
    }

    #[test]
    fn reports_the_size_a_fixed_up_tree_needs() {
        install_hooks();
        let original = blob(false);
        let mut buffer = original.clone();
        let mut size = buffer.len();
        assert_eq!(
            call(&mut buffer, &mut size, APPLY_FIXUPS),
            Status::BUFFER_TOO_SMALL
        );
        assert!(size > original.len());
        assert_eq!(buffer, original, "buffer must be untouched");
    }

    #[test]
    fn applies_fixups_in_place() {
        install_hooks();
        let mut buffer = blob(false);
        let mut size = 4096;
        buffer.resize(size, 0);
        assert_eq!(call(&mut buffer, &mut size, APPLY_FIXUPS), Status::SUCCESS);
        let tree = Tree::parse(&buffer).unwrap();
        assert_eq!(
            tree.node("/fixed")
                .and_then(|node| node.property("compatible")),
            Some(&b"crabefi,test\0"[..])
        );
        assert_eq!(
            apply_platform_fixups(&buffer),
            None,
            "fixups are idempotent"
        );
    }

    #[test]
    fn leaves_an_unchanged_tree_byte_for_byte() {
        install_hooks();
        let original = blob(true);
        let mut buffer = original.clone();
        let mut size = buffer.len();
        assert_eq!(call(&mut buffer, &mut size, APPLY_FIXUPS), Status::SUCCESS);
        assert_eq!(buffer, original);
        assert_eq!(size, original.len());
    }
}
