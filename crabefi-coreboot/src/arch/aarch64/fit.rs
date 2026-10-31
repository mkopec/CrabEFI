//! Devicetree handoff when coreboot starts CrabEFI from a FIT.
//!
//! coreboot's FIT loader treats the payload as a Linux arm64 Image: it
//! selects the configuration whose devicetree matches the board/SKU, adds
//! memory, a `simple-framebuffer` under `/chosen` and `/firmware/coreboot`
//! (whose first `reg` range is the coreboot table), and enters with x0 = the
//! devicetree. ELF payloads instead receive x0 = the coreboot table.

const FDT_MAGIC: u32 = 0xd00d_feed;
/// The devicetree is copied here: coreboot places it in RAM that CrabEFI's
/// page allocator would otherwise hand out.
const MAX_FDT: usize = 512 * 1024;

#[repr(C, align(8))]
struct FdtCopy([u8; MAX_FDT]);
static mut FDT_COPY: FdtCopy = FdtCopy([0; MAX_FDT]);

/// Result of interpreting the payload entry argument.
pub struct Handoff {
    /// Pointer for the coreboot table parser.
    pub coreboot_table: u64,
    /// Devicetree copy, when entered from a FIT.
    pub fdt: Option<&'static [u8]>,
}

/// Interpret x0 at entry.
///
/// # Safety
/// `argument` must be the unmodified x0 value coreboot passed to the
/// payload; it is only dereferenced when it points into identity-mapped RAM.
pub unsafe fn handoff(argument: u64) -> Handoff {
    let legacy = Handoff {
        coreboot_table: argument,
        fdt: None,
    };
    if argument == 0 || !argument.is_multiple_of(4) {
        return legacy;
    }
    // SAFETY: coreboot passes either the coreboot table or the FDT, both in RAM.
    let header = unsafe { core::slice::from_raw_parts(argument as *const u8, 8) };
    let magic = u32::from_be_bytes([header[0], header[1], header[2], header[3]]);
    let size = u32::from_be_bytes([header[4], header[5], header[6], header[7]]) as usize;
    if magic != FDT_MAGIC {
        return legacy;
    }
    if size > MAX_FDT {
        log::error!("FIT devicetree too large ({size} bytes); ignoring it");
        return Handoff {
            coreboot_table: 0,
            fdt: None,
        };
    }
    // SAFETY: the FDT header announced `size` bytes; the copy is written
    // once, before anything else reads it.
    let copy: &'static [u8] = unsafe {
        let destination = core::ptr::addr_of_mut!(FDT_COPY) as *mut u8;
        core::ptr::copy_nonoverlapping(argument as *const u8, destination, size);
        core::slice::from_raw_parts(destination, size)
    };
    let coreboot_table = coreboot_table_from_fdt(copy).unwrap_or_else(|| {
        log::warn!("FIT devicetree has no /firmware/coreboot reg; scanning for tables");
        0
    });
    Handoff {
        coreboot_table,
        fdt: Some(copy),
    }
}

fn coreboot_table_from_fdt(blob: &[u8]) -> Option<u64> {
    let dt = fdt::Fdt::new(blob).ok()?;
    let node = dt.find_node("/firmware/coreboot")?;
    let first = node.reg()?.next()?;
    Some(first.starting_address as u64)
}
