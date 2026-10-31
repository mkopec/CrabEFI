//! MT8196 board description from the devicetree.
//!
//! When coreboot starts CrabEFI from a FIT it passes the board's Linux
//! devicetree. The MT8196 drivers take their device topology from it:
//! controller register windows, the I2C peripherals with their interrupt
//! lines, HID descriptor registers, supplies and power-on delays, pin groups,
//! and the PMIC regulators marked `regulator-always-on`. SoC knowledge (PMIC
//! register layouts, pad fields, interrupt controller wiring, clock gates)
//! stays in the drivers; the devicetree only selects what to use.
//!
//! Without a usable devicetree (an ELF payload, or a FIT without a board
//! DTB) the description is empty and the MT8196 drivers stay unused.

use alloc::{string::String, vec::Vec};
use crabefi::cell::Local;

/// One I2C peripheral (a child node of an I2C controller).
#[derive(Clone, Debug)]
pub struct I2cDevice {
    pub compatible: Vec<String>,
    pub address: u8,
    /// `hid-descr-addr` of a HID-over-I2C device.
    pub hid_descriptor: Option<u16>,
    /// GPIO number and IRQ_TYPE_* flags from `interrupts-extended` on pio.
    pub interrupt: Option<(u32, u32)>,
    /// Node name of the supply regulator (`vdd-supply`/`vcc-supply`).
    pub supply: Option<String>,
    pub post_power_on_delay_ms: u32,
}

impl I2cDevice {
    pub fn is_compatible(&self, compatible: &str) -> bool {
        self.compatible.iter().any(|c| c == compatible)
    }
}

#[derive(Clone, Debug)]
pub struct I2cBus {
    pub base: u64,
    pub dma: u64,
    pub frequency: u32,
    /// `pinmux` cells and their bias/drive settings.
    pub pins: Vec<PinGroup>,
    pub devices: Vec<I2cDevice>,
}

/// A `pins-*` node: pins (`pin << 8 | function`) sharing configuration.
#[derive(Clone, Debug)]
pub struct PinGroup {
    pub pinmux: Vec<u32>,
    pub bias_disable: bool,
    /// Drive strength in mA.
    pub drive_strength: Option<u32>,
}

/// A PMIC regulator the OS keeps on (`regulator-always-on`).
#[derive(Clone, Debug)]
pub struct Regulator {
    /// Node name, e.g. `vcn33-2`.
    pub name: String,
    pub microvolt: u32,
}

#[derive(Clone, Debug, Default)]
pub struct Board {
    pub i2c: Vec<I2cBus>,
    pub ufs: Option<u64>,
    /// SPI controller of the ChromeOS EC with the keyboard.
    pub ec_spi: Option<u64>,
    /// `linux,keymap` of the EC keyboard (`row << 24 | column << 16 | code`).
    pub ec_keymap: Vec<u32>,
    /// MT6373 always-on regulators.
    pub mt6373_always_on: Vec<Regulator>,
}

static BOARD: Local<Option<Board>> = Local::new(None);

/// Describe the board from `fdt`.
pub fn init(fdt: Option<&[u8]>) {
    let board = match fdt.map(fdt::Fdt::new) {
        Some(Ok(dt)) => parse(&dt),
        Some(Err(error)) => {
            log::warn!("MT8196: unusable devicetree ({error:?}); no board devices");
            Board::default()
        }
        None => {
            log::warn!("MT8196: no devicetree from coreboot; no board devices");
            Board::default()
        }
    };
    log::info!(
        "MT8196 board: {} I2C buses, UFS {:x?}, EC SPI {:x?}",
        board.i2c.len(),
        board.ufs,
        board.ec_spi,
    );
    *BOARD.borrow_mut() = Some(board);
}

/// The board description (empty before `init()`).
pub fn board() -> Board {
    BOARD.borrow().clone().unwrap_or_default()
}

pub const IRQ_TYPE_EDGE_RISING: u32 = 1;
pub const IRQ_TYPE_LEVEL_LOW: u32 = 8;

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

type Node<'b, 'a> = fdt::node::FdtNode<'b, 'a>;

fn cells(node: Node<'_, '_>, name: &str) -> Vec<u32> {
    node.property(name).map_or_else(Vec::new, |property| {
        property
            .value
            .as_chunks::<4>()
            .0
            .iter()
            .map(|cell| u32::from_be_bytes(*cell))
            .collect()
    })
}

fn u32_property(node: Node<'_, '_>, name: &str) -> Option<u32> {
    cells(node, name).first().copied()
}

fn okay(node: Node<'_, '_>) -> bool {
    node.property("status")
        .and_then(|status| status.as_str())
        .is_none_or(|status| status == "okay" || status == "ok")
}

fn compatible_with(node: Node<'_, '_>, compatible: &str) -> bool {
    node.compatible()
        .is_some_and(|list| list.all().any(|value| value == compatible))
}

/// `reg` of a node directly under a 2-address/2-size-cell bus.
fn regs64(node: Node<'_, '_>) -> Vec<(u64, u64)> {
    cells(node, "reg")
        .as_chunks::<4>()
        .0
        .iter()
        .map(|c| {
            (
                (u64::from(c[0]) << 32) | u64::from(c[1]),
                (u64::from(c[2]) << 32) | u64::from(c[3]),
            )
        })
        .collect()
}

fn parse(dt: &fdt::Fdt<'_>) -> Board {
    let mut board = Board::default();
    let pio = dt
        .all_nodes()
        .find(|node| compatible_with(*node, "mediatek,mt8196-pinctrl"))
        .and_then(|node| u32_property(node, "phandle"));

    for node in dt.all_nodes().filter(|node| okay(*node)) {
        if compatible_with(node, "mediatek,mt8188-i2c") {
            if let Some(bus) = parse_i2c(dt, node, pio) {
                board.i2c.push(bus);
            }
        } else if compatible_with(node, "mediatek,mt8196-ufshci") {
            board.ufs = regs64(node).first().map(|&(base, _)| base);
        } else if node.children().any(|child| {
            // The EC with the keyboard, not the fingerprint MCU (also cros-ec-spi).
            compatible_with(child, "google,cros-ec-spi")
                && child
                    .children()
                    .any(|grandchild| compatible_with(grandchild, "google,cros-ec-keyb"))
        }) {
            board.ec_spi = regs64(node).first().map(|&(base, _)| base);
            board.ec_keymap = node
                .children()
                .flat_map(|ec| ec.children())
                .find(|child| compatible_with(*child, "google,cros-ec-keyb"))
                .map(|keyboard| cells(keyboard, "linux,keymap"))
                .unwrap_or_default();
        } else if compatible_with(node, "mediatek,mt6373-regulator") {
            board.mt6373_always_on = node
                .children()
                .filter(|regulator| regulator.property("regulator-always-on").is_some())
                .filter_map(|regulator| {
                    Some(Regulator {
                        name: String::from(regulator.name),
                        microvolt: u32_property(regulator, "regulator-min-microvolt")?,
                    })
                })
                .collect();
        }
    }
    board
}

fn parse_i2c(dt: &fdt::Fdt<'_>, node: Node<'_, '_>, pio: Option<u32>) -> Option<I2cBus> {
    let regs = regs64(node);
    let (&(base, _), &(dma, _)) = (regs.first()?, regs.get(1)?);
    let pins = cells(node, "pinctrl-0")
        .into_iter()
        .filter_map(|phandle| dt.find_phandle(phandle))
        .flat_map(|config| config.children().map(pin_group).collect::<Vec<_>>())
        .collect();
    let devices = node
        .children()
        .filter(|child| okay(*child))
        .filter_map(|child| {
            let address = u8::try_from(u32_property(child, "reg")?).ok()?;
            let interrupt = match cells(child, "interrupts-extended").as_slice() {
                [parent, gpio, flags] if Some(*parent) == pio => Some((*gpio, *flags)),
                _ => None,
            };
            let supply = ["vdd-supply", "vcc-supply"]
                .iter()
                .find_map(|name| u32_property(child, name))
                .and_then(|phandle| dt.find_phandle(phandle))
                .map(|regulator| String::from(regulator.name));
            Some(I2cDevice {
                compatible: child
                    .compatible()
                    .map(|list| list.all().map(String::from).collect())
                    .unwrap_or_default(),
                address,
                hid_descriptor: u32_property(child, "hid-descr-addr")
                    .and_then(|value| u16::try_from(value).ok()),
                interrupt,
                supply,
                post_power_on_delay_ms: u32_property(child, "post-power-on-delay-ms").unwrap_or(0),
            })
        })
        .collect();
    Some(I2cBus {
        base,
        dma,
        frequency: u32_property(node, "clock-frequency").unwrap_or(100_000),
        pins,
        devices,
    })
}

fn pin_group(node: Node<'_, '_>) -> PinGroup {
    PinGroup {
        pinmux: cells(node, "pinmux"),
        bias_disable: node.property("bias-disable").is_some(),
        drive_strength: u32_property(node, "drive-strength"),
    }
}

#[cfg(test)]
mod tests;
