//! Editable flattened devicetree.
//!
//! Firmware devicetree fixups (`EFI_DT_FIXUP_PROTOCOL`) add nodes and
//! properties to a devicetree supplied by an OS loader. A blob is parsed into
//! an owned tree, edited, and serialized again as a version 17 FDT.

use alloc::string::String;
use alloc::vec::Vec;

const MAGIC: u32 = 0xd00d_feed;
const HEADER_SIZE: usize = 40;
const TOKEN_BEGIN_NODE: u32 = 1;
const TOKEN_END_NODE: u32 = 2;
const TOKEN_PROP: u32 = 3;
const TOKEN_NOP: u32 = 4;
const TOKEN_END: u32 = 9;
/// Deepest node nesting accepted from a blob.
const MAX_DEPTH: usize = 64;
/// Most memory reservation entries accepted from a blob.
const MAX_RESERVATIONS: usize = 1024;

/// A devicetree property.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Property {
    pub name: String,
    pub value: Vec<u8>,
}

/// A devicetree node with its properties and subnodes.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Node {
    /// Node name including any unit address (`name@unit`); empty for the root.
    pub name: String,
    pub properties: Vec<Property>,
    pub children: Vec<Node>,
}

/// A parsed devicetree.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Tree {
    pub root: Node,
    /// `/memreserve/` entries as (address, size).
    pub reservations: Vec<(u64, u64)>,
    boot_cpuid: u32,
}

/// Why [`Tree::copy_node`] could not copy a node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CopyError {
    /// The node or its parent is missing from the source tree.
    NotFound,
    /// The source node's `reg` does not match its parent's cell sizes.
    InvalidReg,
    /// The target parent does not state its cell sizes, but other children
    /// rely on them, so they cannot be added.
    UnknownCells,
    /// An address or size does not fit the target parent's cell sizes.
    DoesNotFit,
    /// The target cannot reproduce the source parent's address translation.
    Unsupported,
}

fn be32(bytes: &[u8], offset: usize) -> Option<u32> {
    Some(u32::from_be_bytes(
        bytes.get(offset..offset.checked_add(4)?)?.try_into().ok()?,
    ))
}

fn be64(bytes: &[u8], offset: usize) -> Option<u64> {
    Some(u64::from_be_bytes(
        bytes.get(offset..offset.checked_add(8)?)?.try_into().ok()?,
    ))
}

fn align4(value: usize) -> Option<usize> {
    Some(value.checked_add(3)? & !3)
}

/// Read a `cells`-cell big-endian number (0 to 2 cells).
pub fn read_cells(bytes: &[u8], cells: u32) -> Option<u64> {
    match cells {
        0 if bytes.is_empty() => Some(0),
        1 => Some(u64::from(be32(bytes, 0).filter(|_| bytes.len() == 4)?)),
        2 => be64(bytes, 0).filter(|_| bytes.len() == 8),
        _ => None,
    }
}

/// Encode a number in `cells` big-endian cells, if it fits.
pub fn write_cells(out: &mut Vec<u8>, value: u64, cells: u32) -> Option<()> {
    match cells {
        0 if value == 0 => {}
        1 => out.extend_from_slice(&u32::try_from(value).ok()?.to_be_bytes()),
        2 => out.extend_from_slice(&value.to_be_bytes()),
        _ => return None,
    }
    Some(())
}

impl Node {
    /// A node without properties or children.
    pub fn new(name: &str) -> Self {
        Self {
            name: name.into(),
            ..Self::default()
        }
    }

    pub fn property(&self, name: &str) -> Option<&[u8]> {
        self.properties
            .iter()
            .find(|property| property.name == name)
            .map(|property| property.value.as_slice())
    }

    pub fn u32_property(&self, name: &str) -> Option<u32> {
        self.property(name)
            .filter(|value| value.len() == 4)
            .and_then(|value| be32(value, 0))
    }

    /// Add a property or replace its value.
    pub fn set_property(&mut self, name: &str, value: &[u8]) {
        match self
            .properties
            .iter_mut()
            .find(|property| property.name == name)
        {
            Some(property) => property.value = value.into(),
            None => self.properties.push(Property {
                name: name.into(),
                value: value.into(),
            }),
        }
    }

    pub fn child(&self, name: &str) -> Option<&Node> {
        self.children.iter().find(|child| child.name == name)
    }

    pub fn child_mut(&mut self, name: &str) -> Option<&mut Node> {
        self.children.iter_mut().find(|child| child.name == name)
    }

    /// `#address-cells` and `#size-cells` stated by this node, with the
    /// devicetree specification defaults (2 and 1) for missing ones.
    pub fn cells(&self) -> (u32, u32) {
        (
            self.u32_property("#address-cells").unwrap_or(2),
            self.u32_property("#size-cells").unwrap_or(1),
        )
    }

    /// Decode a `reg`-like property of a child of this node.
    pub fn decode_reg(&self, value: &[u8]) -> Option<Vec<(u64, u64)>> {
        let (address_cells, size_cells) = self.cells();
        if address_cells > 2 || size_cells > 2 {
            return None;
        }
        let split = address_cells as usize * 4;
        let entry = split + size_cells as usize * 4;
        if entry == 0 || !value.len().is_multiple_of(entry) {
            return None;
        }
        value
            .chunks_exact(entry)
            .map(|chunk| {
                Some((
                    read_cells(&chunk[..split], address_cells)?,
                    read_cells(&chunk[split..], size_cells)?,
                ))
            })
            .collect()
    }
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

fn components(path: &str) -> impl Iterator<Item = &str> {
    path.split('/').filter(|component| !component.is_empty())
}

impl Tree {
    /// Parse a version 17 FDT that passes [`validate`]; `None` if it is
    /// malformed or deeper than the parser supports.
    pub fn parse(blob: &[u8]) -> Option<Self> {
        let blob = blob.get(..validate(blob)?)?;
        let header =
            |index: usize| -> Option<usize> { usize::try_from(be32(blob, index * 4)?).ok() };
        let (off_struct, off_strings, off_reservations) = (header(2)?, header(3)?, header(4)?);
        let (size_strings, size_struct) = (header(8)?, header(9)?);
        let strings = blob.get(off_strings..off_strings.checked_add(size_strings)?)?;
        let structure = blob.get(off_struct..off_struct.checked_add(size_struct)?)?;

        let mut reservations = Vec::new();
        for index in 0..=MAX_RESERVATIONS {
            let offset = off_reservations.checked_add(index * 16)?;
            let (address, size) = (be64(blob, offset)?, be64(blob, offset + 8)?);
            if address == 0 && size == 0 {
                break;
            }
            if index == MAX_RESERVATIONS {
                return None;
            }
            address.checked_add(size)?;
            reservations.push((address, size));
        }

        let root = parse_structure(structure, strings)?;
        Some(Self {
            root,
            reservations,
            boot_cpuid: be32(blob, 28)?,
        })
    }

    /// Serialize as a version 17 FDT.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut structure = Vec::new();
        let mut strings = Vec::new();
        write_node(&self.root, &mut structure, &mut strings);
        structure.extend_from_slice(&TOKEN_END.to_be_bytes());

        let off_reservations = HEADER_SIZE;
        let off_struct = off_reservations + (self.reservations.len() + 1) * 16;
        let off_strings = off_struct + structure.len();
        let total = off_strings + strings.len();
        let mut blob = Vec::with_capacity(total);
        for field in [
            MAGIC,
            total as u32,
            off_struct as u32,
            off_strings as u32,
            off_reservations as u32,
            17,
            16,
            self.boot_cpuid,
            strings.len() as u32,
            structure.len() as u32,
        ] {
            blob.extend_from_slice(&field.to_be_bytes());
        }
        for &(address, size) in self.reservations.iter().chain(&[(0, 0)]) {
            blob.extend_from_slice(&address.to_be_bytes());
            blob.extend_from_slice(&size.to_be_bytes());
        }
        blob.extend_from_slice(&structure);
        blob.extend_from_slice(&strings);
        blob
    }

    /// The node at an absolute path (`/` is the root).
    pub fn node(&self, path: &str) -> Option<&Node> {
        components(path).try_fold(&self.root, |node, name| node.child(name))
    }

    pub fn node_mut(&mut self, path: &str) -> Option<&mut Node> {
        components(path).try_fold(&mut self.root, |node, name| node.child_mut(name))
    }

    /// Copy the node at `path`, with its subtree, from `source` if this tree
    /// does not have it yet. Returns whether the node was added.
    ///
    /// The node's `reg` is re-encoded for the cell sizes of its parent here.
    /// A missing parent (only directly below the root) is created, a parent
    /// without cell sizes gets the source parent's, and an identity `ranges`
    /// in the source parent is reproduced, so `reg` translates as it does in
    /// the source.
    pub fn copy_node(&mut self, source: &Tree, path: &str) -> Result<bool, CopyError> {
        if self.node(path).is_some() {
            return Ok(false);
        }
        let (parent_path, _) = path.rsplit_once('/').ok_or(CopyError::NotFound)?;
        let source_parent = source.node(parent_path).ok_or(CopyError::NotFound)?;
        let mut node = source.node(path).ok_or(CopyError::NotFound)?.clone();
        let reg = match node.property("reg") {
            Some(value) => Some(
                source_parent
                    .decode_reg(value)
                    .ok_or(CopyError::InvalidReg)?,
            ),
            None => None,
        };

        let parent = match self.node_mut(parent_path) {
            Some(parent) => parent,
            None => {
                let mut ancestors = components(parent_path);
                let (Some(name), None) = (ancestors.next(), ancestors.next()) else {
                    return Err(CopyError::Unsupported);
                };
                self.root.children.push(Node::new(name));
                self.root.children.last_mut().ok_or(CopyError::NotFound)?
            }
        };
        let is_root = components(parent_path).next().is_none();
        let (address_cells, size_cells) = match (
            parent.u32_property("#address-cells"),
            parent.u32_property("#size-cells"),
        ) {
            (Some(address_cells), Some(size_cells)) => (address_cells, size_cells),
            _ if parent
                .children
                .iter()
                .any(|child| child.property("reg").is_some()) =>
            {
                return Err(CopyError::UnknownCells);
            }
            _ => source_parent.cells(),
        };
        let mut encoded = Vec::new();
        for (address, size) in reg.iter().flatten() {
            write_cells(&mut encoded, *address, address_cells).ok_or(CopyError::DoesNotFit)?;
            write_cells(&mut encoded, *size, size_cells).ok_or(CopyError::DoesNotFit)?;
        }
        if !is_root && parent.property("ranges").is_none() {
            match source_parent.property("ranges") {
                Some([]) => parent.set_property("ranges", &[]),
                Some(_) => return Err(CopyError::Unsupported),
                None => {}
            }
        }

        parent.set_property("#address-cells", &address_cells.to_be_bytes());
        parent.set_property("#size-cells", &size_cells.to_be_bytes());
        if reg.is_some() {
            node.set_property("reg", &encoded);
        }
        parent.children.push(node);
        Ok(true)
    }
}

fn parse_structure(structure: &[u8], strings: &[u8]) -> Option<Node> {
    let mut stack: Vec<Node> = Vec::new();
    let mut root = None;
    let mut position = 0usize;
    loop {
        let token = be32(structure, position)?;
        position += 4;
        match token {
            TOKEN_BEGIN_NODE => {
                let rest = structure.get(position..)?;
                let length = rest.iter().position(|&byte| byte == 0)?;
                let name = core::str::from_utf8(&rest[..length]).ok()?;
                position = align4(position.checked_add(length + 1)?)?;
                if root.is_some()
                    || stack.len() >= MAX_DEPTH
                    || (stack.is_empty() && !name.is_empty())
                {
                    return None;
                }
                stack.push(Node::new(name));
            }
            TOKEN_END_NODE => {
                let node = stack.pop()?;
                match stack.last_mut() {
                    Some(parent) => parent.children.push(node),
                    None => root = Some(node),
                }
            }
            TOKEN_PROP => {
                let length = usize::try_from(be32(structure, position)?).ok()?;
                let name_offset = usize::try_from(be32(structure, position + 4)?).ok()?;
                position = position.checked_add(8)?;
                let value = structure.get(position..position.checked_add(length)?)?;
                position = align4(position + length)?;
                let name = strings.get(name_offset..)?;
                let name = &name[..name.iter().position(|&byte| byte == 0)?];
                stack.last_mut()?.properties.push(Property {
                    name: core::str::from_utf8(name).ok()?.into(),
                    value: value.into(),
                });
            }
            TOKEN_NOP => {}
            TOKEN_END if stack.is_empty() => return root,
            _ => return None,
        }
    }
}

fn string_offset(strings: &mut Vec<u8>, name: &str) -> u32 {
    let mut offset = 0;
    for existing in strings.split(|&byte| byte == 0) {
        if existing == name.as_bytes() && offset < strings.len() {
            return offset as u32;
        }
        offset += existing.len() + 1;
    }
    let offset = strings.len();
    strings.extend_from_slice(name.as_bytes());
    strings.push(0);
    offset as u32
}

fn pad4(out: &mut Vec<u8>) {
    out.resize(out.len().next_multiple_of(4), 0);
}

fn write_node(node: &Node, out: &mut Vec<u8>, strings: &mut Vec<u8>) {
    out.extend_from_slice(&TOKEN_BEGIN_NODE.to_be_bytes());
    out.extend_from_slice(node.name.as_bytes());
    out.push(0);
    pad4(out);
    for property in &node.properties {
        out.extend_from_slice(&TOKEN_PROP.to_be_bytes());
        out.extend_from_slice(&(property.value.len() as u32).to_be_bytes());
        out.extend_from_slice(&string_offset(strings, &property.name).to_be_bytes());
        out.extend_from_slice(&property.value);
        pad4(out);
    }
    for child in &node.children {
        write_node(child, out, strings);
    }
    out.extend_from_slice(&TOKEN_END_NODE.to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    fn u32s(values: &[u32]) -> Vec<u8> {
        values
            .iter()
            .flat_map(|value| value.to_be_bytes())
            .collect()
    }

    fn coreboot_source() -> Tree {
        let mut coreboot = Node::new("coreboot");
        coreboot.set_property("compatible", b"coreboot\0");
        coreboot.set_property(
            "reg",
            &u32s(&[0xfe00_0000, 0x1000, 0xfd00_0000, 0x0200_0000]),
        );
        coreboot.set_property("board-id", &u32s(&[3]));
        let mut firmware = Node::new("firmware");
        firmware.set_property("ranges", &[]);
        firmware.set_property("#address-cells", &u32s(&[1]));
        firmware.set_property("#size-cells", &u32s(&[1]));
        firmware.children.push(coreboot);
        let mut root = Node::new("");
        root.children.push(firmware);
        Tree {
            root,
            reservations: vec![(0x8000_0000, 0x1000)],
            boot_cpuid: 0,
        }
    }

    fn target() -> Tree {
        let mut firmware = Node::new("firmware");
        let mut optee = Node::new("optee");
        optee.set_property("compatible", b"linaro,optee-tz\0");
        firmware.children.push(optee);
        let mut root = Node::new("");
        root.set_property("#address-cells", &u32s(&[2]));
        root.set_property("#size-cells", &u32s(&[2]));
        root.set_property("compatible", b"vendor,board\0");
        root.children.push(firmware);
        Tree {
            root,
            reservations: Vec::new(),
            boot_cpuid: 0,
        }
    }

    #[test]
    fn serialization_round_trips() {
        for tree in [coreboot_source(), target()] {
            let blob = tree.to_bytes();
            assert_eq!(Tree::parse(&blob), Some(tree.clone()));
            assert_eq!(Tree::parse(&blob).unwrap().to_bytes(), blob);
        }
    }

    #[test]
    fn every_truncation_and_bad_header_is_rejected() {
        let blob = coreboot_source().to_bytes();
        for length in 0..blob.len() {
            assert_eq!(Tree::parse(&blob[..length]), None, "length {length}");
        }
        for (offset, value) in [(0, 0u32), (20, 16), (24, 18), (8, u32::MAX), (12, u32::MAX)] {
            let mut bad = blob.clone();
            bad[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
            assert_eq!(Tree::parse(&bad), None, "offset {offset}");
        }
    }

    #[test]
    fn copies_node_into_existing_parent_with_cells_and_ranges() {
        let mut tree = target();
        assert_eq!(
            tree.copy_node(&coreboot_source(), "/firmware/coreboot"),
            Ok(true)
        );
        let firmware = tree.node("/firmware").unwrap();
        assert_eq!(firmware.cells(), (1, 1));
        assert_eq!(firmware.property("ranges"), Some(&[][..]));
        let coreboot = tree.node("/firmware/coreboot").unwrap();
        assert_eq!(coreboot.property("board-id"), Some(&u32s(&[3])[..]));
        assert_eq!(
            firmware.decode_reg(coreboot.property("reg").unwrap()),
            Some(vec![(0xfe00_0000, 0x1000), (0xfd00_0000, 0x0200_0000)])
        );
        // Existing children stay; a second copy is a no-op.
        assert!(tree.node("/firmware/optee").is_some());
        let before = tree.clone();
        assert_eq!(
            tree.copy_node(&coreboot_source(), "/firmware/coreboot"),
            Ok(false)
        );
        assert_eq!(tree, before);
        assert_eq!(Tree::parse(&tree.to_bytes()), Some(tree));
    }

    #[test]
    fn copies_into_missing_parent_and_reencodes_for_existing_cells() {
        let mut tree = target();
        tree.root.children.clear();
        assert_eq!(
            tree.copy_node(&coreboot_source(), "/firmware/coreboot"),
            Ok(true)
        );
        assert!(tree.node("/firmware/coreboot").is_some());

        let mut tree = target();
        let firmware = tree.node_mut("/firmware").unwrap();
        firmware.set_property("#address-cells", &u32s(&[2]));
        firmware.set_property("#size-cells", &u32s(&[2]));
        firmware.set_property("ranges", &[]);
        assert_eq!(
            tree.copy_node(&coreboot_source(), "/firmware/coreboot"),
            Ok(true)
        );
        let firmware = tree.node("/firmware").unwrap();
        let reg = tree
            .node("/firmware/coreboot")
            .unwrap()
            .property("reg")
            .unwrap();
        assert_eq!(reg.len(), 32);
        assert_eq!(
            firmware.decode_reg(reg).unwrap()[1],
            (0xfd00_0000, 0x0200_0000)
        );
    }

    #[test]
    fn refuses_ambiguous_or_lossy_copies() {
        let mut tree = target();
        tree.node_mut("/firmware/optee")
            .unwrap()
            .set_property("reg", &u32s(&[0, 1, 0]));
        assert_eq!(
            tree.copy_node(&coreboot_source(), "/firmware/coreboot"),
            Err(CopyError::UnknownCells)
        );

        let mut source = coreboot_source();
        source
            .node_mut("/firmware")
            .unwrap()
            .set_property("#address-cells", &u32s(&[2]));
        source
            .node_mut("/firmware/coreboot")
            .unwrap()
            .set_property("reg", &u32s(&[1, 0, 0x1000]));
        let mut tree = target();
        let firmware = tree.node_mut("/firmware").unwrap();
        firmware.set_property("#address-cells", &u32s(&[1]));
        firmware.set_property("#size-cells", &u32s(&[1]));
        assert_eq!(
            tree.copy_node(&source, "/firmware/coreboot"),
            Err(CopyError::DoesNotFit)
        );

        source
            .node_mut("/firmware/coreboot")
            .unwrap()
            .set_property("reg", &u32s(&[1, 0]));
        assert_eq!(
            target().copy_node(&source, "/firmware/coreboot"),
            Err(CopyError::InvalidReg)
        );
        assert_eq!(
            target().copy_node(&source, "/firmware/missing"),
            Err(CopyError::NotFound)
        );
    }

    /// Validate and round-trip real devicetrees when provided, e.g.
    /// `CRABEFI_TEST_DTB=board.dtb`; several paths are separated by `:`.
    #[test]
    fn round_trips_external_devicetree() {
        let Some(paths) = std::env::var_os("CRABEFI_TEST_DTB") else {
            return;
        };
        for path in std::env::split_paths(&paths) {
            let blob = std::fs::read(&path).unwrap();
            assert!(validate(&blob).is_some(), "{}", path.display());
            let tree = Tree::parse(&blob).expect("parse");
            let rewritten = tree.to_bytes();
            assert_eq!(validate(&rewritten), Some(rewritten.len()));
            assert_eq!(Tree::parse(&rewritten), Some(tree));
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
