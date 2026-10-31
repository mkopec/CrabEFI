//! Platform facts the coreboot table does not carry, read from the
//! devicetree coreboot passes with a FIT payload.

/// UART register interface, from the compatible of the devicetree node at
/// the coreboot serial record's address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartKind {
    /// 8250/16550-compatible (Linux 8250_of and its vendor variants).
    Ns16550,
    /// ARM PrimeCell PL011.
    Pl011,
}

/// Compatibles of 16550-compatible UARTs (Linux 8250_of, 8250_mtk, 8250_dw).
const NS16550_COMPATIBLES: &[&str] = &[
    "ns8250",
    "ns16450",
    "ns16550",
    "ns16550a",
    "ns16750",
    "mediatek,mt6577-uart",
    "snps,dw-apb-uart",
];

/// The kind of the UART whose first `reg` range starts at `base`, if the
/// devicetree describes it with a known compatible.
pub fn uart_kind(fdt: &[u8], base: u64) -> Option<UartKind> {
    let dt = fdt::Fdt::new(fdt).ok()?;
    let node = dt.all_nodes().find(|node| {
        node.reg()
            .and_then(|mut reg| reg.next())
            .is_some_and(|region| region.starting_address as u64 == base)
    })?;
    let compatible = node.compatible()?;
    if compatible.all().any(|c| NS16550_COMPATIBLES.contains(&c)) {
        Some(UartKind::Ns16550)
    } else if compatible.all().any(|c| c == "arm,pl011") {
        Some(UartKind::Pl011)
    } else {
        None
    }
}

/// Whether any device is described as DMA-coherent. Linux treats devices
/// without `dma-coherent` as not snooping the CPU caches; a tree in which no
/// device is coherent describes a SoC whose DMA masters all need
/// non-cacheable buffers.
pub fn any_dma_coherent(fdt: &[u8]) -> bool {
    fdt::Fdt::new(fdt).is_ok_and(|dt| {
        dt.all_nodes()
            .any(|node| node.property("dma-coherent").is_some())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const SOC_NONCOHERENT: &[u8] = include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/soc-noncoherent.dtb"
    ));
    const PL011_COHERENT: &[u8] = include_bytes!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/pl011-coherent.dtb"
    ));

    #[test]
    fn uart_kind_comes_from_the_node_at_the_serial_base() {
        assert_eq!(
            uart_kind(SOC_NONCOHERENT, 0x1600_0000),
            Some(UartKind::Ns16550)
        );
        assert_eq!(
            uart_kind(PL011_COHERENT, 0x0900_0000),
            Some(UartKind::Pl011)
        );
        // No node at that address, or an unknown compatible.
        assert_eq!(uart_kind(SOC_NONCOHERENT, 0x1600_1000), None);
        assert_eq!(uart_kind(SOC_NONCOHERENT, 0x1601_0000), None);
        assert_eq!(uart_kind(PL011_COHERENT, 0x0910_0000), None);
        assert_eq!(uart_kind(&[0u8; 16], 0x1600_0000), None);
    }

    #[test]
    fn coherency_needs_a_dma_coherent_device() {
        assert!(!any_dma_coherent(SOC_NONCOHERENT));
        assert!(any_dma_coherent(PL011_COHERENT));
        assert!(!any_dma_coherent(&[0u8; 16]));
    }
}
