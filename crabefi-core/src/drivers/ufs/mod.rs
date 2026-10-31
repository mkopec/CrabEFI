//! UFS host controller (UFSHCI) driver, ported from Depthcharge's ufs.c.
//!
//! The platform powers and clocks the controller and passes its register
//! base with the vendor [`Quirks`] it needs. One transfer slot, polled, no
//! task management. Logical units are exposed READ-ONLY as platform block
//! devices.
//!
//! DMA uses identity addresses from [`crate::efi::allocate_dma_pages`]: on
//! platforms whose controller does not snoop the CPU caches, the platform
//! must register a non-cacheable DMA pool first.

use alloc::{boxed::Box, format, string::String};

use crate::barrier;
use crate::drivers::block::{BlockDevice, BlockDeviceInfo, BlockError};
use crate::drivers::mmio::MmioRegion;
use crate::time::{Timeout, delay_ms, delay_us};

/// Size of the standard UFSHCI register file.
const REGISTERS: usize = 0x100;

/// Vendor deviations from the standard UFSHCI initialization flow.
#[derive(Debug, Clone, Copy, Default)]
pub struct Quirks {
    /// Clear `PA_Local_TX_LCC_Enable` before link startup (MediaTek).
    pub disable_tx_lcc: bool,
    /// Switch to HS-G1 in FASTAUTO mode before the final high-speed gear
    /// switch on UniPro 1.8 and later (MediaTek `mtk_ufs_pmc_via_fastauto`).
    pub fastauto_before_hs: bool,
}

const VER: u64 = 0x08;
const IS: u64 = 0x20;
const HCS: u64 = 0x30;
const HCE: u64 = 0x34;
const UECPA: u64 = 0x38;
const UECDL: u64 = 0x3c;
const UECN: u64 = 0x40;
const UECT: u64 = 0x44;
const UECDME: u64 = 0x48;
const UTRLBA: u64 = 0x50;
const UTRLBAU: u64 = 0x54;
const UTRLDBR: u64 = 0x58;
const UTRLCLR: u64 = 0x5c;
const UTRLRSR: u64 = 0x60;
const UICCMD: u64 = 0x90;
const UICCMDARG1: u64 = 0x94;
const UICCMDARG2: u64 = 0x98;
const UICCMDARG3: u64 = 0x9c;

const IS_UTRCS: u32 = 1 << 0;
const IS_UE: u32 = 1 << 2;
const IS_UPMS: u32 = 1 << 4;
const IS_ULLS: u32 = 1 << 7;
const IS_ULSS: u32 = 1 << 8;
const IS_UCCS: u32 = 1 << 10;
const IS_DFES: u32 = 1 << 11;
const IS_HCFES: u32 = 1 << 16;
const IS_SBFES: u32 = 1 << 17;
const IS_FATAL: u32 = IS_ULLS | IS_DFES | IS_HCFES | IS_SBFES;
const IS_ALL_ERROR: u32 = IS_UE | IS_FATAL;
const HCS_DP: u32 = 1 << 0;
const HCS_UCRDY: u32 = 1 << 3;
const UECDL_PA_INIT_ERROR: u32 = 1 << 13;

const DME_GET: u32 = 0x01;
const DME_SET: u32 = 0x02;
const DME_PEER_GET: u32 = 0x03;
const DME_PEER_SET: u32 = 0x04;
const DME_LINKSTARTUP: u32 = 0x16;

const PA_ACTIVETXDATALANES: u32 = 0x1560;
const PA_ACTIVERXDATALANES: u32 = 0x1580;
const PA_TXTRAILINGCLOCKS: u32 = 0x1564;
const PA_MINRXTRAILINGCLOCKS: u32 = 0x1543;
const PA_LOCAL_TX_LCC_ENABLE: u32 = 0x155e;
const PA_TXGEAR: u32 = 0x1568;
const PA_TXTERMINATION: u32 = 0x1569;
const PA_HSSERIES: u32 = 0x156a;
const PA_PWRMODE: u32 = 0x1571;
const PA_RXGEAR: u32 = 0x1583;
const PA_RXTERMINATION: u32 = 0x1584;
const PA_MAXRXPWMGEAR: u32 = 0x1586;
const PA_MAXRXHSGEAR: u32 = 0x1587;
const PA_LOCALVERINFO: u32 = 0x15a9;
const PA_CONNECTEDTXDATALANES: u32 = 0x1561;
const PA_CONNECTEDRXDATALANES: u32 = 0x1581;
const PA_TXHSADAPTTYPE: u32 = 0x15d4;
const PA_INITIAL_ADAPT: u32 = 1;
const PA_NO_ADAPT: u32 = 3;
const DME_LOCALFC0PROTECTIONTIMEOUTVAL: u32 = 0xd041;
const DME_LOCALTC0REPLAYTIMEOUTVAL: u32 = 0xd042;
const DME_LOCALAFC0REQTIMEOUTVAL: u32 = 0xd043;

const FAST_MODE: u32 = 1;
const SLOW_MODE: u32 = 2;
const FASTAUTO_MODE: u32 = 4;
const HS_SERIES_B: u32 = 2;
const UNIPRO_1_8: u32 = 5;

const UPIU_NOP_OUT: u8 = 0x00;
const UPIU_NOP_IN: u8 = 0x20;
const UPIU_COMMAND: u8 = 0x01;
const UPIU_RESPONSE: u8 = 0x21;
const UPIU_QUERY_REQ: u8 = 0x16;
const UPIU_QUERY_RESP: u8 = 0x36;
const QUERY_READ_DESC: u8 = 0x1;
const QUERY_READ_FLAG: u8 = 0x5;
const QUERY_SET_FLAG: u8 = 0x6;
const IDN_FDEVICEINIT: u8 = 0x01;
const DESC_DEVICE: u8 = 0x00;
const DESC_UNIT: u8 = 0x02;

// Memory: 32 UTRDs (1 KiB), then tag 0's UCD: command UPIU, response UPIU, PRDT.
const REQ_LIST_SZ: usize = 1024;
const UPIU_SZ: usize = 512;
const RESP_OFFS: usize = UPIU_SZ;
const PRDT_OFFS: usize = 2 * UPIU_SZ;
const PRDT_ENTRIES: usize = 64;
const PRDT_DBC_MAX: usize = 0x40000;
const MEM_SZ: usize = REQ_LIST_SZ + PRDT_OFFS + PRDT_ENTRIES * 16;
const BOUNCE_SZ: usize = 1 << 20;
const MAX_LUN: u8 = 32;

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
enum UfsError {
    Timeout(&'static str),
    Fatal(u32),
    LinkFatal,
    Uic(u32, u32, u32),
    Ocs(u8),
    Protocol(&'static str),
    Response(u8),
    Scsi(u8),
    Sense(u8),
    UnitAttention,
    NoMemory,
}

/// Register access and UIC commands.
struct Hci(MmioRegion);

impl Hci {
    fn r32(&self, offset: u64) -> u32 {
        self.0.read32(offset)
    }

    fn w32(&self, offset: u64, value: u32) {
        self.0.write32(offset, value);
    }

    fn wait(&self, reg: u64, mask: u32, value: u32, timeout_us: u64) -> bool {
        let timeout = Timeout::from_us(timeout_us);
        loop {
            let expired = timeout.is_expired();
            if self.r32(reg) & mask == value {
                return true;
            }
            if expired {
                return false;
            }
        }
    }

    fn poll_completion(
        &self,
        mask: u32,
        doorbell: u32,
        timeout_us: u64,
        what: &'static str,
    ) -> Result<(), UfsError> {
        let timeout = Timeout::from_us(timeout_us);
        loop {
            let expired = timeout.is_expired();
            let status = self.r32(IS);
            if status & mask != 0 && (doorbell == 0 || self.r32(UTRLDBR) & doorbell == 0) {
                self.w32(IS, mask);
                return Ok(());
            }
            if expired {
                return Err(UfsError::Timeout(what));
            }
            if status & IS_ALL_ERROR != 0 {
                self.w32(IS, IS_ALL_ERROR);
                let (pa, dl, n, t, dme) = (
                    self.r32(UECPA),
                    self.r32(UECDL),
                    self.r32(UECN),
                    self.r32(UECT),
                    self.r32(UECDME),
                );
                log::debug!(
                    "UFS: IS={status:#x} UECPA={pa:#x} UECDL={dl:#x} UECN={n:#x} UECT={t:#x} UECDME={dme:#x}"
                );
                if status & IS_FATAL != 0 {
                    return Err(UfsError::Fatal(status));
                }
                if dl & UECDL_PA_INIT_ERROR != 0 {
                    return Err(UfsError::LinkFatal);
                }
            }
        }
    }

    fn uic_once(&self, cmd: u32, attr: u32, set: u32) -> Result<u32, UfsError> {
        if !self.wait(HCS, HCS_UCRDY, HCS_UCRDY, 100_000) {
            return Err(UfsError::Timeout("UCRDY"));
        }
        self.w32(IS, IS_UCCS);
        self.w32(UICCMDARG1, attr);
        self.w32(UICCMDARG2, 0); // normal attribute set type / reserved
        self.w32(UICCMDARG3, set);
        self.w32(UICCMD, cmd);
        self.poll_completion(IS_UCCS, 0, 100_000, "UIC")?;
        let result = self.r32(UICCMDARG2) & 0xff;
        if result != 0 {
            return Err(UfsError::Uic(cmd, attr >> 16, result));
        }
        Ok(self.r32(UICCMDARG3))
    }

    fn uic(&self, cmd: u32, attr: u32, set: u32) -> Result<u32, UfsError> {
        let retries = if cmd == DME_PEER_GET || cmd == DME_PEER_SET {
            3
        } else {
            1
        };
        let mut result = self.uic_once(cmd, attr, set);
        for _ in 1..retries {
            if result.is_ok() {
                break;
            }
            result = self.uic_once(cmd, attr, set);
        }
        result
    }

    fn dme_get(&self, id: u32) -> Result<u32, UfsError> {
        self.uic(DME_GET, id << 16, 0)
    }

    fn dme_peer_get(&self, id: u32) -> Result<u32, UfsError> {
        self.uic(DME_PEER_GET, id << 16, 0)
    }

    fn dme_set(&self, id: u32, value: u32) -> Result<(), UfsError> {
        self.uic(DME_SET, id << 16, value).map(|_| ())
    }

    fn dme_peer_set(&self, id: u32, value: u32) -> Result<(), UfsError> {
        self.uic(DME_PEER_SET, id << 16, value).map(|_| ())
    }
}

#[derive(Clone, Copy, Default, Debug)]
struct Lane {
    lanes: u32,
    gear: u32,
    mode: u32,
}

struct Ufs {
    hci: Hci,
    quirks: Quirks,
    mem: u64,
    bounce: u64,
    unipro: u32,
    tx: Lane,
    rx: Lane,
}

impl Ufs {
    fn utrd(&self) -> *mut u8 {
        self.mem as *mut u8
    }
    fn ucd(&self) -> u64 {
        self.mem + REQ_LIST_SZ as u64
    }
    fn ucd_ptr(&self, offset: usize) -> *mut u8 {
        (self.ucd() + offset as u64) as *mut u8
    }

    /// Fill tag 0's UTRD. `ddir`: 0 none, 1 write, 2 read.
    fn prepare_utrd(&self, ddir: u8, prdt_len: u16) {
        let mut d = [0u8; 32];
        d[3] = (1 << 4) | (ddir << 1) | 1; // CT=UFS storage, DD, I=normal
        d[8] = 0x0f; // OCS: invalid, overwritten by the controller
        d[16..20].copy_from_slice(&(self.ucd() as u32).to_le_bytes());
        d[20..24].copy_from_slice(&((self.ucd() >> 32) as u32).to_le_bytes());
        d[24..26].copy_from_slice(&((UPIU_SZ / 4) as u16).to_le_bytes());
        d[26..28].copy_from_slice(&((RESP_OFFS / 4) as u16).to_le_bytes());
        d[28..30].copy_from_slice(&prdt_len.to_le_bytes());
        d[30..32].copy_from_slice(&((PRDT_OFFS / 4) as u16).to_le_bytes());
        // SAFETY: tag 0's UTRD within the owned, NC request list.
        unsafe { core::ptr::copy_nonoverlapping(d.as_ptr(), self.utrd(), 32) };
    }

    fn ocs(&self) -> u8 {
        // SAFETY: as above.
        unsafe { core::ptr::read_volatile(self.utrd().add(8)) & 0x0f }
    }

    fn run(&self) -> Result<(), UfsError> {
        let hci = &self.hci;
        hci.w32(IS, IS_UE | IS_UTRCS);
        if hci.r32(UTRLRSR) == 0 {
            return Err(UfsError::Protocol("UTRLRSR not set"));
        }
        // Complete the descriptor stores before ringing the doorbell.
        barrier::mmio_write();
        hci.w32(UTRLDBR, 1);
        if let Err(error) = hci.poll_completion(IS_UTRCS, 1, 30_000_000, "transfer") {
            hci.w32(UTRLCLR, !1);
            return Err(error);
        }
        barrier::consume_from_device();
        match self.ocs() {
            0 => Ok(()),
            ocs => Err(UfsError::Ocs(ocs)),
        }
    }

    fn clear_upius(&self) {
        // SAFETY: command and response UPIU areas of tag 0.
        unsafe { core::ptr::write_bytes(self.ucd_ptr(0), 0, 2 * UPIU_SZ) };
    }

    fn resp(&self, offset: usize) -> u8 {
        // SAFETY: response UPIU area of tag 0.
        unsafe { core::ptr::read_volatile(self.ucd_ptr(RESP_OFFS + offset)) }
    }

    fn nop(&self) -> Result<(), UfsError> {
        self.clear_upius();
        let c = self.ucd_ptr(0);
        unsafe { c.write_volatile(UPIU_NOP_OUT) };
        self.prepare_utrd(0, 0);
        self.run()?;
        if self.resp(0) != UPIU_NOP_IN {
            return Err(UfsError::Protocol("NOP IN"));
        }
        Ok(())
    }

    /// Query request; returns (value, response data length) and copies data.
    fn query(&self, op: u8, idn: u8, idx: u8, data: Option<&mut [u8]>) -> Result<u32, UfsError> {
        self.clear_upius();
        let mut c = [0u8; 32];
        c[0] = UPIU_QUERY_REQ;
        c[5] = if matches!(op, QUERY_READ_DESC | QUERY_READ_FLAG | 0x3) {
            0x01
        } else {
            0x81
        };
        c[12] = op;
        c[13] = idn;
        c[14] = idx;
        let len = data.as_ref().map_or(0, |d| d.len().min(255)) as u16;
        c[18..20].copy_from_slice(&len.to_be_bytes());
        // SAFETY: command UPIU area.
        unsafe { core::ptr::copy_nonoverlapping(c.as_ptr(), self.ucd_ptr(0), 32) };
        self.prepare_utrd(0, 0);
        self.run()?;
        if self.resp(0) != UPIU_QUERY_RESP {
            return Err(UfsError::Protocol("query response type"));
        }
        if self.resp(12) != op || self.resp(6) != 0 {
            return Err(UfsError::Response(self.resp(6)));
        }
        match op {
            QUERY_READ_DESC => {
                let got = usize::from(u16::from_be_bytes([self.resp(18), self.resp(19)]));
                if let Some(data) = data {
                    let n = got.min(data.len());
                    for (i, byte) in data[..n].iter_mut().enumerate() {
                        *byte = self.resp(32 + i);
                    }
                }
                Ok(got as u32)
            }
            QUERY_READ_FLAG => Ok(u32::from(self.resp(23))),
            _ => Ok(0),
        }
    }

    fn descriptor(&self, idn: u8, idx: u8, buf: &mut [u8]) -> Result<usize, UfsError> {
        buf.fill(0);
        let len = self.query(QUERY_READ_DESC, idn, idx, Some(buf))? as usize;
        if buf[1] != idn {
            return Err(UfsError::Protocol("descriptor IDN"));
        }
        Ok(len)
    }

    /// SCSI command with optional data phase through the DMA bounce buffer.
    fn scsi(&self, lun: u8, cdb: &[u8], read_len: usize) -> Result<(), UfsError> {
        let mut attempts = 0;
        loop {
            match self.scsi_once(lun, cdb, read_len) {
                Err(UfsError::UnitAttention) if attempts < 3 => attempts += 1,
                result => return result,
            }
        }
    }

    fn scsi_once(&self, lun: u8, cdb: &[u8], read_len: usize) -> Result<(), UfsError> {
        if read_len > BOUNCE_SZ || !read_len.is_multiple_of(4) {
            return Err(UfsError::Protocol("transfer length"));
        }
        self.clear_upius();
        let mut c = [0u8; 32];
        c[0] = UPIU_COMMAND;
        c[1] = if read_len != 0 { 0x40 } else { 0 };
        c[2] = lun;
        c[12..16].copy_from_slice(&(read_len as u32).to_be_bytes());
        c[16..16 + cdb.len()].copy_from_slice(cdb);
        // SAFETY: command UPIU area.
        unsafe { core::ptr::copy_nonoverlapping(c.as_ptr(), self.ucd_ptr(0), 32) };
        let mut entries = 0u16;
        let mut remaining = read_len;
        let mut address = self.bounce;
        while remaining > 0 {
            let count = remaining.min(PRDT_DBC_MAX);
            let mut entry = [0u8; 16];
            entry[0..8].copy_from_slice(&address.to_le_bytes());
            entry[12..16].copy_from_slice(&(count as u32 - 1).to_le_bytes());
            // SAFETY: PRDT entries within the allocation (BOUNCE_SZ fits).
            unsafe {
                core::ptr::copy_nonoverlapping(
                    entry.as_ptr(),
                    self.ucd_ptr(PRDT_OFFS + 16 * usize::from(entries)),
                    16,
                );
            }
            entries += 1;
            address += count as u64;
            remaining -= count;
        }
        self.prepare_utrd(if read_len != 0 { 2 } else { 0 }, entries);
        self.run()?;
        if self.resp(0) != UPIU_RESPONSE {
            return Err(UfsError::Protocol("response type"));
        }
        let status = self.resp(7);
        let data_len = u16::from_be_bytes([self.resp(10), self.resp(11)]);
        if status == 0x02 && data_len != 0 {
            // Sense data: 2-byte length, then fixed-format sense.
            let key = self.resp(32 + 2 + 2) & 0x0f;
            return Err(if key == 0x6 {
                UfsError::UnitAttention
            } else {
                UfsError::Sense(key)
            });
        }
        if self.resp(6) != 0 {
            return Err(UfsError::Response(self.resp(6)));
        }
        if status != 0 {
            return Err(UfsError::Scsi(status));
        }
        Ok(())
    }

    fn link_up(&mut self) -> Result<(), UfsError> {
        let hci = &self.hci;
        hci.w32(HCE, 0);
        if !hci.wait(HCE, 1, 0, 10_000) {
            return Err(UfsError::Timeout("HCE clear"));
        }
        hci.w32(HCE, 1);
        delay_ms(1);
        if !hci.wait(HCE, 1, 1, 100_000) {
            return Err(UfsError::Timeout("HCE set"));
        }
        if self.quirks.disable_tx_lcc {
            hci.dme_set(PA_LOCAL_TX_LCC_ENABLE, 0)?;
        }
        let mut result = Err(UfsError::Timeout("link startup"));
        for attempt in 1..=5 {
            if let Err(error) = hci.uic(DME_LINKSTARTUP, 0, 0) {
                log::warn!("UFS: DME_LINKSTARTUP attempt {attempt}: {error:?}");
                result = Err(error);
                continue;
            }
            if hci.wait(HCS, HCS_DP, HCS_DP, 10_000) {
                result = Ok(());
                break;
            }
            hci.wait(IS, IS_ULSS, IS_ULSS, 10_000);
            hci.w32(IS, IS_ULSS);
            log::warn!("UFS: no device after link startup attempt {attempt}");
        }
        result?;
        // SAFETY: owned, page-aligned (>= 1 KiB) NC allocation below 4 GiB.
        unsafe { core::ptr::write_bytes(self.mem as *mut u8, 0, MEM_SZ) };
        hci.w32(UTRLBA, self.mem as u32);
        hci.w32(UTRLBAU, (self.mem >> 32) as u32);
        hci.w32(UTRLRSR, 1);
        Ok(())
    }

    fn device_init(&self) -> Result<(), UfsError> {
        let mut nop = Err(UfsError::Protocol("NOP"));
        for _ in 0..10 {
            nop = self.nop();
            if nop.is_ok() {
                break;
            }
        }
        nop?;
        self.query(QUERY_SET_FLAG, IDN_FDEVICEINIT, 0, None)?;
        let timeout = Timeout::from_ms(5000);
        loop {
            let expired = timeout.is_expired();
            if self.query(QUERY_READ_FLAG, IDN_FDEVICEINIT, 0, None)? == 0 {
                return Ok(());
            }
            if expired {
                return Err(UfsError::Timeout("fDeviceInit"));
            }
            delay_ms(1);
        }
    }

    fn power_mode_change(&self, tx: Lane, rx: Lane) -> Result<(), UfsError> {
        let hci = &self.hci;
        let fast = |mode| mode == FAST_MODE || mode == FASTAUTO_MODE;
        hci.w32(IS, IS_UPMS);
        hci.dme_set(PA_ACTIVETXDATALANES, tx.lanes)?;
        hci.dme_set(PA_ACTIVERXDATALANES, rx.lanes)?;
        hci.dme_set(PA_RXGEAR, rx.gear)?;
        hci.dme_set(PA_TXGEAR, tx.gear)?;
        hci.dme_set(PA_TXTERMINATION, u32::from(fast(tx.mode)))?;
        hci.dme_set(PA_RXTERMINATION, u32::from(fast(rx.mode)))?;
        if fast(tx.mode) || fast(rx.mode) {
            hci.dme_set(PA_HSSERIES, HS_SERIES_B)?;
        }
        for (index, value) in [8191, 65535, 32767, 8191, 65535, 32767]
            .into_iter()
            .enumerate()
        {
            hci.dme_set(0x15b0 + index as u32, value)?;
        }
        hci.dme_set(DME_LOCALFC0PROTECTIONTIMEOUTVAL, 8191)?;
        hci.dme_set(DME_LOCALTC0REPLAYTIMEOUTVAL, 65535)?;
        hci.dme_set(DME_LOCALAFC0REQTIMEOUTVAL, 32767)?;
        hci.dme_set(PA_PWRMODE, rx.mode << 4 | tx.mode)?;
        hci.poll_completion(IS_UPMS, 0, 500_000, "power mode change")?;
        let upmcrs = (hci.r32(HCS) >> 8) & 7;
        if upmcrs != 1 {
            return Err(UfsError::Uic(PA_PWRMODE, 0, upmcrs));
        }
        Ok(())
    }

    fn gear_switch(&mut self) -> Result<(), UfsError> {
        self.rx.lanes = self.hci.dme_get(PA_CONNECTEDRXDATALANES)?;
        self.tx.lanes = self.hci.dme_get(PA_CONNECTEDTXDATALANES)?;
        self.rx.gear = self.hci.dme_get(PA_MAXRXHSGEAR)?;
        self.tx.gear = self.hci.dme_peer_get(PA_MAXRXHSGEAR)?;
        self.rx.mode = if self.rx.gear != 0 {
            FAST_MODE
        } else {
            SLOW_MODE
        };
        if self.rx.gear == 0 {
            self.rx.gear = self.hci.dme_get(PA_MAXRXPWMGEAR)?;
        }
        self.tx.mode = if self.tx.gear != 0 {
            FAST_MODE
        } else {
            SLOW_MODE
        };
        if self.tx.gear == 0 {
            self.tx.gear = self.hci.dme_peer_get(PA_MAXRXPWMGEAR)?;
        }
        if self.rx.lanes == 0 || self.tx.lanes == 0 || self.rx.gear == 0 || self.tx.gear == 0 {
            return Err(UfsError::Protocol("lanes/gears"));
        }
        log::info!(
            "UFS: target tx {:?} rx {:?}, UniPro {}",
            self.tx,
            self.rx,
            self.unipro
        );
        // Same condition as Depthcharge's mtk_ufs_pmc_via_fastauto().
        let fastauto_first = self.quirks.fastauto_before_hs
            && self.unipro >= UNIPRO_1_8
            && self.tx.mode == FAST_MODE
            && self.rx.mode == FAST_MODE;
        if fastauto_first {
            let hci = &self.hci;
            let value = hci.dme_peer_get(PA_MINRXTRAILINGCLOCKS)?;
            hci.dme_set(PA_TXTRAILINGCLOCKS, value)?;
            let value = hci.dme_get(PA_MINRXTRAILINGCLOCKS)?;
            hci.dme_peer_set(PA_TXTRAILINGCLOCKS, value)?;
            hci.dme_set(PA_TXHSADAPTTYPE, PA_NO_ADAPT)?;
            let g1 = |lane: Lane| Lane {
                gear: 1,
                mode: FASTAUTO_MODE,
                ..lane
            };
            self.power_mode_change(g1(self.tx), g1(self.rx))?;
        }
        if self.unipro >= UNIPRO_1_8 {
            let adapt = if self.rx.mode == FAST_MODE && self.tx.gear >= 4 {
                PA_INITIAL_ADAPT
            } else {
                PA_NO_ADAPT
            };
            self.hci.dme_set(PA_TXHSADAPTTYPE, adapt)?;
        }
        self.power_mode_change(self.tx, self.rx)
    }

    fn setup(&mut self, high_speed: bool) -> Result<u8, UfsError> {
        self.link_up()?;
        self.device_init()?;
        let mut device = [0u8; 255];
        self.descriptor(DESC_DEVICE, 0, &mut device)?;
        self.unipro = self.hci.dme_get(PA_LOCALVERINFO)?;
        log::info!(
            "UFS: HCI {:#x}, UniPro {}, spec {:#06x}, {} LUs",
            self.hci.r32(VER),
            self.unipro,
            u16::from_be_bytes([device[16], device[17]]),
            device[6]
        );
        if device[6] != 0 && high_speed {
            self.gear_switch()?;
            log::info!("UFS: gear switch done");
        }
        Ok(device[6])
    }
}

struct UfsLun {
    ufs: &'static Ufs,
    lun: u8,
    block_size: u32,
    blocks: u64,
    name: String,
}

impl BlockDevice for UfsLun {
    fn info(&self) -> BlockDeviceInfo {
        BlockDeviceInfo {
            num_blocks: self.blocks,
            block_size: self.block_size,
            media_id: 0,
            removable: false,
            read_only: true,
        }
    }

    fn read_blocks(&mut self, lba: u64, count: u32, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.validate_io(lba, count, buffer)?;
        let size = self.block_size as usize;
        let per_command = (BOUNCE_SZ / size) as u64;
        let (mut lba, mut left, mut offset) = (lba, u64::from(count), 0usize);
        while left > 0 {
            let blocks = left.min(per_command);
            if lba + blocks > u64::from(u32::MAX) {
                return Err(BlockError::OutOfRange);
            }
            let bytes = blocks as usize * size;
            let mut cdb = [0u8; 16];
            cdb[0] = 0x28; // READ(10)
            cdb[1] = 0x08; // FUA, as Depthcharge
            cdb[2..6].copy_from_slice(&(lba as u32).to_be_bytes());
            cdb[7..9].copy_from_slice(&(blocks as u16).to_be_bytes());
            self.ufs.scsi(self.lun, &cdb, bytes).map_err(|error| {
                log::error!(
                    "UFS LUN {}: READ(10) lba {lba} x{blocks}: {error:?}",
                    self.lun
                );
                BlockError::DeviceError
            })?;
            // SAFETY: the controller filled `bytes` of the NC bounce buffer.
            unsafe {
                core::ptr::copy_nonoverlapping(
                    self.ufs.bounce as *const u8,
                    buffer[offset..].as_mut_ptr(),
                    bytes,
                );
            }
            lba += blocks;
            left -= blocks;
            offset += bytes;
        }
        Ok(())
    }

    fn name(&self) -> &str {
        &self.name
    }
}

/// Bring up the UFS controller at `base` and register its logical units as
/// read-only platform block devices. Returns the number registered.
///
/// # Safety
///
/// `base` must be the identity-mapped (Device memory) register base of a
/// UFSHCI the platform has already powered, clocked and taken out of reset,
/// and nothing else may drive that controller.
pub unsafe fn init(base: u64, quirks: Quirks) -> usize {
    // SAFETY: the caller guarantees a mapped, exclusively owned UFSHCI.
    let regs = match unsafe { MmioRegion::try_new(base, REGISTERS) } {
        Ok(regs) => regs,
        Err(error) => {
            log::error!("UFS: invalid controller base {base:#x}: {error:?}");
            return 0;
        }
    };
    let hci = Hci(regs);
    let cap = hci.r32(0);
    if cap == 0 || cap == u32::MAX {
        log::error!("UFS: controller not accessible (CAP={cap:#x}); skipping");
        return 0;
    }
    let (Some(mem), Some(bounce)) = (
        crate::efi::allocate_dma_pages(MEM_SZ.div_ceil(4096) as u64),
        crate::efi::allocate_dma_pages((BOUNCE_SZ / 4096) as u64),
    ) else {
        log::error!("UFS: {:?}", UfsError::NoMemory);
        return 0;
    };
    log::info!(
        "UFS: CAP={cap:#x} VER={:#x} HCS={:#x}",
        hci.r32(VER),
        hci.r32(HCS)
    );
    let mut ufs = Ufs {
        hci,
        quirks,
        mem: mem.as_ptr() as u64,
        bounce: bounce.as_ptr() as u64,
        unipro: 0,
        tx: Lane::default(),
        rx: Lane::default(),
    };

    // Depthcharge retries setup five times. Fall back to the power-on PWM
    // gear if high-speed switching keeps failing: slow, but bootable.
    let mut luns = None;
    for attempt in 0..5 {
        let high_speed = attempt < 3;
        match ufs.setup(high_speed) {
            Ok(count) => {
                luns = Some(count);
                if !high_speed {
                    log::warn!("UFS: running without a high-speed gear switch (slow)");
                }
                break;
            }
            Err(error) => log::warn!(
                "UFS: setup attempt {} (high speed {high_speed}) failed: {error:?}",
                attempt + 1
            ),
        }
        delay_us(1000);
    }
    let Some(lu_count) = luns else {
        log::error!("UFS: initialization failed");
        ufs.hci.w32(HCE, 0);
        return 0;
    };
    let ufs: &'static Ufs = Box::leak(Box::new(ufs));

    let (mut found, mut registered) = (0, 0);
    for lun in 0..MAX_LUN {
        if found >= lu_count {
            break;
        }
        let mut unit = [0u8; 255];
        if ufs.descriptor(DESC_UNIT, lun, &mut unit).is_err() || unit[3] == 0 {
            continue;
        }
        found += 1;
        let mut ready = ufs.scsi(lun, &[0u8; 6], 0); // TEST UNIT READY
        if ready.is_err() {
            ready = ufs.scsi(lun, &[0u8; 6], 0);
        }
        if let Err(error) = ready {
            log::warn!("UFS LUN {lun}: not ready: {error:?}");
            continue;
        }
        let Some((block_size, blocks)) = unit_geometry(&unit) else {
            log::warn!("UFS LUN {lun}: bad geometry");
            continue;
        };
        log::info!(
            "UFS LUN {lun}: {blocks} x {block_size} bytes ({} MiB), read-only",
            (blocks * u64::from(block_size)) >> 20
        );
        let device: &'static mut dyn BlockDevice = Box::leak(Box::new(UfsLun {
            ufs,
            lun,
            block_size,
            blocks,
            name: format!("UFS LUN {lun}"),
        }));
        if crate::drivers::storage::add_platform_block_device(device) {
            registered += 1;
        } else {
            log::error!("UFS LUN {lun}: platform block device table full");
        }
    }
    registered
}

/// Block size and count from a unit descriptor (`bLogicalBlockSize` as a
/// power of two at byte 10, `qLogicalBlockCount` at bytes 11..19).
fn unit_geometry(unit: &[u8; 255]) -> Option<(u32, u64)> {
    let shift = unit[10];
    let blocks = u64::from_be_bytes(unit[11..19].try_into().ok()?);
    ((9..=16).contains(&shift) && blocks != 0).then(|| (1u32 << shift, blocks))
}

#[cfg(test)]
mod tests {
    use super::unit_geometry;

    #[test]
    fn unit_geometry_validates_descriptor() {
        let mut unit = [0u8; 255];
        unit[10] = 12;
        unit[11..19].copy_from_slice(&62_486_528u64.to_be_bytes());
        assert_eq!(unit_geometry(&unit), Some((4096, 62_486_528)));
        unit[10] = 8;
        assert_eq!(unit_geometry(&unit), None);
        unit[10] = 17;
        assert_eq!(unit_geometry(&unit), None);
        unit[10] = 9;
        unit[11..19].fill(0);
        assert_eq!(unit_geometry(&unit), None);
    }
}
