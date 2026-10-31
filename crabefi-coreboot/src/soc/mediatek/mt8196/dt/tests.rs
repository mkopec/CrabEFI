use alloc::vec;
use alloc::vec::Vec;

use super::*;

const BOARD_DTB: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/tests/fixtures/mt8196-board.dtb"
));

fn board() -> Board {
    parse(&fdt::Fdt::new(BOARD_DTB).unwrap())
}

#[test]
fn i2c_buses_skip_disabled_nodes_and_keep_device_wiring() {
    let board = board();
    let buses: Vec<_> = board
        .i2c
        .iter()
        .map(|bus| (bus.base, bus.dma, bus.frequency))
        .collect();
    assert_eq!(
        buses,
        [
            (0x1313_0000, 0x1637_0000, 400_000),
            (0x1393_0000, 0x1638_0000, 1_000_000)
        ]
    );

    let touchpads = &board.i2c[0].devices;
    assert_eq!(touchpads.len(), 2);
    let hid = touchpads.iter().find(|d| d.address == 0x38).unwrap();
    assert!(hid.is_compatible("hid-over-i2c"));
    assert_eq!(hid.hid_descriptor, Some(0x20));
    assert_eq!(hid.interrupt, Some((8, IRQ_TYPE_LEVEL_LOW)));
    assert_eq!(hid.supply.as_deref(), Some("vcn33-2"));
    assert_eq!(hid.post_power_on_delay_ms, 200);
    let elan = touchpads.iter().find(|d| d.address == 0x15).unwrap();
    assert_eq!(elan.supply.as_deref(), Some("vcn33-2"));
    assert_eq!(elan.hid_descriptor, None);

    let tpm = &board.i2c[1].devices[0];
    assert!(tpm.is_compatible("google,cr50"));
    assert_eq!(tpm.interrupt, Some((18, IRQ_TYPE_EDGE_RISING)));
    assert_eq!(tpm.supply, None);
}

#[test]
fn pin_groups_and_always_on_regulators() {
    let board = board();
    let pins: Vec<_> = board.i2c[0]
        .pins
        .iter()
        .map(|g| (g.pinmux.clone(), g.bias_disable, g.drive_strength))
        .collect();
    assert_eq!(
        pins,
        [
            (vec![0x6301, 0x6401], true, Some(4)),
            (vec![0x0800], true, None)
        ]
    );
    assert!(board.i2c[1].pins.is_empty());
    let regulators: Vec<_> = board
        .mt6373_always_on
        .iter()
        .map(|r| (r.name.as_str(), r.microvolt))
        .collect();
    assert_eq!(regulators, [("vfp", 1_800_000), ("vcn33-2", 3_300_000)]);
}

#[test]
fn missing_or_invalid_devicetree_describes_no_devices() {
    init(None);
    let empty = super::board();
    assert!(empty.i2c.is_empty() && empty.mt6373_always_on.is_empty());
    init(Some(&[0u8; 64]));
    assert!(super::board().i2c.is_empty());
    init(Some(BOARD_DTB));
    assert_eq!(super::board().i2c.len(), 2);
}
