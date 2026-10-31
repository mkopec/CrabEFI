//! Flattened devicetree (FDT) validation.
//!
//! Devicetrees from disk (BLS `devicetree` entries) are checked before use:
//! [`validate`] bounds every read by the blob's header without allocating.

const MAGIC: u32 = 0xd00d_feed;
const HEADER_SIZE: usize = 40;
const TOKEN_BEGIN_NODE: u32 = 1;
const TOKEN_END_NODE: u32 = 2;
const TOKEN_PROP: u32 = 3;
const TOKEN_NOP: u32 = 4;
const TOKEN_END: u32 = 9;

fn be32(bytes: &[u8], offset: usize) -> Option<u32> {
    Some(u32::from_be_bytes(
        bytes.get(offset..offset.checked_add(4)?)?.try_into().ok()?,
    ))
}

fn overlaps(a: core::ops::Range<usize>, b: core::ops::Range<usize>) -> bool {
    a.start < b.end && b.start < a.end
}

/// Validate a version 17 compatible FDT without allocating and return its
/// total size, excluding any padding after it (disk files). Every read is
/// bounded by the header's size; blocks must be aligned and disjoint, and
/// the structure block must be a single well-nested root.
pub fn validate(bytes: &[u8]) -> Option<usize> {
    if be32(bytes, 0)? != MAGIC || bytes.len() < HEADER_SIZE {
        return None;
    }
    let total = be32(bytes, 4)? as usize;
    let bytes = bytes.get(..total)?;
    let structure = be32(bytes, 8)? as usize;
    let strings = be32(bytes, 12)? as usize;
    let reserve = be32(bytes, 16)? as usize;
    if total < HEADER_SIZE
        || be32(bytes, 20)? < 17
        || be32(bytes, 24)? > 17
        || structure < HEADER_SIZE
        || strings < HEADER_SIZE
        || reserve < HEADER_SIZE
        || !structure.is_multiple_of(4)
        || !reserve.is_multiple_of(8)
    {
        return None;
    }
    let strings_end = strings.checked_add(be32(bytes, 32)? as usize)?;
    let structure_end = structure.checked_add(be32(bytes, 36)? as usize)?;
    let names = bytes.get(strings..strings_end)?;
    let tokens = bytes.get(structure..structure_end)?;
    if overlaps(structure..structure_end, strings..strings_end) {
        return None;
    }
    let mut reserve_end = reserve;
    loop {
        let entry = bytes.get(reserve_end..reserve_end.checked_add(16)?)?;
        reserve_end += 16;
        let address = u64::from_be_bytes(entry[..8].try_into().ok()?);
        let size = u64::from_be_bytes(entry[8..].try_into().ok()?);
        address.checked_add(size)?;
        if address == 0 && size == 0 {
            break;
        }
    }
    if overlaps(reserve..reserve_end, structure..structure_end)
        || overlaps(reserve..reserve_end, strings..strings_end)
    {
        return None;
    }

    let mut pos = 0usize;
    let mut depth = 0usize;
    let mut root_seen = false;
    while pos < tokens.len() {
        let token = be32(tokens, pos)?;
        pos += 4;
        match token {
            TOKEN_BEGIN_NODE => {
                let length = tokens.get(pos..)?.iter().position(|b| *b == 0)?;
                if depth == 0 {
                    if root_seen || length != 0 {
                        return None;
                    }
                    root_seen = true;
                }
                pos = pos.checked_add(length)?.checked_add(4)? & !3;
                if pos > tokens.len() {
                    return None;
                }
                depth = depth.checked_add(1)?;
            }
            TOKEN_END_NODE => {
                depth = depth.checked_sub(1)?;
            }
            TOKEN_PROP => {
                if depth == 0 {
                    return None;
                }
                let length = be32(tokens, pos)? as usize;
                let name = be32(tokens, pos + 4)? as usize;
                names.get(name..)?.iter().position(|b| *b == 0)?;
                pos = pos.checked_add(8)?.checked_add(length)?.checked_add(3)? & !3;
                if pos > tokens.len() {
                    return None;
                }
            }
            TOKEN_NOP => {}
            TOKEN_END if depth == 0 && root_seen => {
                return tokens[pos..].iter().all(|b| *b == 0).then_some(total);
            }
            _ => return None,
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Validate real devicetrees when provided, e.g.
    /// `CRABEFI_TEST_DTB=board.dtb`; several paths are separated by `:`.
    #[test]
    fn validates_external_devicetrees() {
        let Some(paths) = std::env::var_os("CRABEFI_TEST_DTB") else {
            return;
        };
        for path in std::env::split_paths(&paths) {
            let blob = std::fs::read(&path).unwrap();
            assert!(validate(&blob).is_some(), "{}", path.display());
        }
    }

    mod validate {
        use super::super::validate;
        fn tree() -> alloc::vec::Vec<u8> {
            let mut data = alloc::vec![0u8; 72];
            for (offset, value) in [
                (0, 0xd00d_feed),
                (4, 72),
                (8, 56),
                (12, 72),
                (16, 40),
                (20, 17),
                (24, 16),
                (36, 16),
                (56, 1),
                (64, 2),
                (68, 9),
            ] {
                data[offset..offset + 4].copy_from_slice(&u32::to_be_bytes(value));
            }
            data
        }
        #[test]
        fn valid_minimal_tree_and_file_padding() {
            let mut data = tree();
            assert_eq!(validate(&data), Some(72));
            data.extend_from_slice(&[0; 16]);
            assert_eq!(validate(&data), Some(72));
        }
        #[test]
        fn every_truncation_is_rejected() {
            let data = tree();
            for len in 0..data.len() {
                assert_eq!(validate(&data[..len]), None);
            }
        }
        #[test]
        fn rejects_bad_offsets_sizes_and_structure() {
            for (offset, value) in [
                (4, u32::MAX),
                (8, 57),
                (8, 0),
                (12, 0),
                (16, 41),
                (16, 56),
                (20, 16),
                (24, 18),
                (32, u32::MAX),
                (36, u32::MAX),
                (56, 3),
                (64, 9),
                (68, 1),
            ] {
                let mut data = tree();
                data[offset..offset + 4].copy_from_slice(&u32::to_be_bytes(value));
                assert_eq!(validate(&data), None, "offset {offset}, value {value}");
            }
        }
    }
}
