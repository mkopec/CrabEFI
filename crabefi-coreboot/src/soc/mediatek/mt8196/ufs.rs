//! MT8196 UFS glue: coreboot's MediaTek FSP powers and clocks the UFSHCI;
//! Depthcharge (Rauru board.c: new_mtk_ufs_ctlr(0x16810000, 0)) needs no
//! device reset (no pericfg base), only the two MediaTek UniPro quirks.

pub fn init() {
    let Some(base) = super::dt::board().ufs else {
        log::info!("UFS: none in the board description");
        return;
    };
    // The Linux ufshci node is neither dma-coherent nor behind an IOMMU.
    if !crabefi::efi::dma_pool::active() {
        log::warn!("UFS: no non-cacheable DMA pool; skipping");
        return;
    }
    let quirks = crabefi::drivers::ufs::Quirks {
        disable_tx_lcc: true,
        fastauto_before_hs: true,
    };
    // SAFETY: the board description's `mediatek,mt8196-ufshci` registers,
    // identity-mapped as Device memory and powered by coreboot.
    unsafe { crabefi::drivers::ufs::init(base, quirks) };
}
