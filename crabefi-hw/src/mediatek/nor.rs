// SPDX-License-Identifier: MIT OR Apache-2.0
//! MediaTek SPI NOR flash controller (SFLASH, `mediatek,mt8186-nor`), polled.
//!
//! Port of coreboot's soc/mediatek/common/flash_controller.c, with Linux
//! spi-mtk-nor's PIO read and buffered page program. Neither allocates nor
//! panics, so a runtime services image can use it after ExitBootServices.
//! Addresses are flash offsets with 3-byte addressing. Clocks, pads, read
//! timing and DMA buffer cache maintenance are the platform's.

use tock_registers::{
    interfaces::{Readable, Writeable},
    register_structs,
    registers::ReadWrite,
};

register_structs! {
    Regs {
        (0x000 => cmd: ReadWrite<u8>),
        (0x001 => _reserved0),
        (0x004 => cnt: ReadWrite<u8>),
        (0x005 => _reserved1),
        (0x008 => rdsr: ReadWrite<u8>),
        (0x009 => _reserved2),
        (0x00c => rdata: ReadWrite<u8>),
        (0x00d => _reserved3),
        (0x010 => radr0: ReadWrite<u8>),
        (0x011 => _reserved4),
        (0x014 => radr1: ReadWrite<u8>),
        (0x015 => _reserved5),
        (0x018 => radr2: ReadWrite<u8>),
        (0x019 => _reserved6),
        (0x01c => wdata: ReadWrite<u8>),
        (0x01d => _reserved7),
        (0x020 => prgdata0: ReadWrite<u8>),
        (0x021 => _reserved8),
        (0x028 => prgdata2: ReadWrite<u8>),
        (0x029 => _reserved9),
        (0x02c => prgdata3: ReadWrite<u8>),
        (0x02d => _reserved10),
        (0x030 => prgdata4: ReadWrite<u8>),
        (0x031 => _reserved11),
        (0x034 => prgdata5: ReadWrite<u8>),
        (0x035 => _reserved12),
        (0x064 => cfg2: ReadWrite<u32>),
        (0x068 => _reserved13),
        (0x098 => pp_data: ReadWrite<u32>),
        (0x09c => _reserved14),
        (0x0c4 => wrprot: ReadWrite<u32>),
        (0x0c8 => _reserved15),
        /// Read mode: bit 0 dual I/O read, bit 4 4-byte addressing.
        (0x0cc => read_mode: ReadWrite<u8>),
        (0x0cd => _reserved16),
        (0x718 => fdma_ctl: ReadWrite<u32>),
        (0x71c => fdma_fadr: ReadWrite<u32>),
        (0x720 => fdma_dadr: ReadWrite<u32>),
        (0x724 => fdma_end_dadr: ReadWrite<u32>),
        (0x728 => @END),
    }
}

const CMD_READ: u8 = 1 << 0;
const CMD_READSTATUS: u8 = 1 << 1;
const CMD_PRG: u8 = 1 << 2;
const CMD_WRITE: u8 = 1 << 4;
const CMD_AUTOINC: u8 = 1 << 7;
const OP_WREN: u8 = 0x06;
const OP_PAGE_PROGRAM: u8 = 0x02;
const OP_SECTOR_ERASE: u8 = 0x20;
const OP_BLOCK_ERASE: u8 = 0xd8;
const OP_READ_1_1_2: u8 = 0x3b;
const READ_MODE_DUAL: u8 = 1 << 0;
const READ_MODE_4B_ADDR: u8 = 1 << 4;
const WR_BUF_EN: u32 = 1 << 0;
const COMMAND_ENABLE: u32 = 0x30;
const DMA_TRIGGER: u32 = 1 << 0;
const DMA_SW_RESET: u32 = 1 << 1;
const DMA_WDLE_EN: u32 = 1 << 2;

pub const SECTOR: u32 = 0x1000;
pub const BLOCK: u32 = 0x10000;
const PAGE: u32 = 0x100;
/// Write buffer size of the controller, as Linux spi-mtk-nor (MTK_NOR_PP_SIZE).
const PP_SIZE: usize = 128;
/// DMA reads move 16-byte aligned flash ranges.
pub const DMA_ALIGN: u32 = 0x10;
/// 3-byte addressing.
pub const MAX_ADDRESS: u32 = 1 << 24;

/// Poll bounds; one status poll is an SPI transaction of at least a
/// microsecond. 64 KiB block erase takes up to 2 s (W25Q64JW tBE1).
const POLLS: u32 = 2_000_000;
const BLOCK_ERASE_POLLS: u32 = 6_000_000;
const CFG_POLLS: u32 = 100_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidRegion,
    /// Misaligned, out of range or overflowing request.
    InvalidArgument,
    Timeout,
}

pub struct MtkNor {
    regs: &'static Regs,
}

// SAFETY: the driver is the controller's only user (see `new`); moving it to
// another thread moves that exclusive access with it.
unsafe impl Send for MtkNor {}

impl MtkNor {
    /// Enable the controller's command interface.
    ///
    /// # Safety
    /// `base` is the controller's register block, clocked and mapped as
    /// device memory, used by nothing else while this driver exists.
    pub unsafe fn new(base: usize) -> Result<Self, Error> {
        if base == 0 || base & 3 != 0 {
            return Err(Error::InvalidRegion);
        }
        // SAFETY: the caller's hardware contract.
        let regs = unsafe { &*(base as *const Regs) };
        regs.wrprot.set(COMMAND_ENABLE);
        Ok(Self { regs })
    }

    /// Use 3-byte addresses (the OS may have switched to 4-byte mode).
    pub fn use_3byte_addresses(&mut self) {
        let mode = self.regs.read_mode.get();
        self.regs.read_mode.set(mode & !READ_MODE_4B_ADDR);
    }

    fn execute(&mut self, cmd: u8) -> Result<(), Error> {
        self.regs.cmd.set(cmd);
        let busy = cmd & !CMD_AUTOINC;
        poll(POLLS, || self.regs.cmd.get() & busy == 0)
    }

    /// Flash status register 1.
    pub fn status(&mut self) -> Result<u8, Error> {
        self.execute(CMD_READSTATUS)?;
        Ok(self.regs.rdsr.get())
    }

    fn wait_ready_polls(&mut self, polls: u32) -> Result<(), Error> {
        for _ in 0..polls {
            if self.status()? & 1 == 0 {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    /// Wait for the flash to finish an erase or program.
    pub fn wait_ready(&mut self) -> Result<(), Error> {
        self.wait_ready_polls(POLLS)
    }

    fn set_address(&mut self, address: u32) {
        let [a0, a1, a2, _] = address.to_le_bytes();
        self.regs.radr0.set(a0);
        self.regs.radr1.set(a1);
        self.regs.radr2.set(a2);
    }

    /// Read through the controller one byte at a time (no DMA buffer).
    pub fn read_pio(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), Error> {
        check(address, buffer.len())?;
        self.wait_ready()?;
        let mut current = address;
        for byte in buffer {
            self.set_address(current);
            self.execute(CMD_READ)?;
            *byte = self.regs.rdata.get();
            current = current.wrapping_add(1);
        }
        Ok(())
    }

    /// Dual I/O DMA read of `len` bytes at the 16-byte aligned `address`
    /// into `destination` (a bus address below 4 GiB). The caller cleans the
    /// destination from the CPU caches before and invalidates it after.
    pub fn read_dma(&mut self, address: u32, destination: u32, len: u32) -> Result<(), Error> {
        let end = destination.checked_add(len).ok_or(Error::InvalidArgument)?;
        if !address.is_multiple_of(DMA_ALIGN) || !len.is_multiple_of(DMA_ALIGN) {
            return Err(Error::InvalidArgument);
        }
        check(address, len as usize)?;
        let regs = self.regs;
        // Erase reuses prgdata[3]; restore the dual read opcode.
        regs.read_mode.set(regs.read_mode.get() | READ_MODE_DUAL);
        regs.prgdata3.set(OP_READ_1_1_2);
        regs.fdma_ctl.set(DMA_SW_RESET);
        regs.fdma_ctl.set(DMA_WDLE_EN);
        regs.fdma_fadr.set(address);
        regs.fdma_dadr.set(destination);
        regs.fdma_end_dadr.set(end);
        regs.fdma_ctl.set(DMA_TRIGGER | DMA_WDLE_EN);
        poll(POLLS, || regs.fdma_ctl.get() & DMA_TRIGGER == 0)
    }

    /// Program one byte at a time with the controller's auto-increment
    /// write, waiting for the flash after each byte.
    pub fn program_bytes(&mut self, address: u32, data: &[u8]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }
        check(address, data.len())?;
        self.wait_ready()?;
        self.set_address(address);
        for &byte in data {
            self.regs.wdata.set(byte);
            self.execute(CMD_WRITE | CMD_AUTOINC)?;
            self.wait_ready()?;
        }
        Ok(())
    }

    /// Switch the controller's page-program buffer (CFG2 WR_BUF_EN).
    fn write_buffer(&mut self, on: bool) -> Result<(), Error> {
        let value = self.regs.cfg2.get();
        self.regs.cfg2.set(if on {
            value | WR_BUF_EN
        } else {
            value & !WR_BUF_EN
        });
        poll(CFG_POLLS, || (self.regs.cfg2.get() & WR_BUF_EN != 0) == on)
    }

    /// Program bytes. Runs of 128 bytes within one 256-byte flash page use the
    /// controller's write buffer (one page program each, as Linux
    /// spi-mtk-nor); the rest is programmed a byte at a time. The buffer is
    /// off again afterwards, as [`MtkNor::program_bytes`] needs.
    pub fn program(&mut self, address: u32, data: &[u8]) -> Result<(), Error> {
        check(address, data.len())?;
        self.regs.prgdata0.set(OP_PAGE_PROGRAM);
        let result = self.program_runs(address, data);
        let disabled = self.write_buffer(false);
        result.and(disabled)
    }

    fn program_runs(&mut self, address: u32, data: &[u8]) -> Result<(), Error> {
        let mut position = address;
        let mut rest = data;
        while let Some(&first) = rest.first() {
            self.wait_ready()?;
            self.set_address(position);
            let run = rest
                .get(..PP_SIZE)
                .and_then(|run| run.as_chunks::<4>().0.get(..PP_SIZE / 4));
            match run {
                Some(words) if position % PAGE + PP_SIZE as u32 <= PAGE => {
                    self.write_buffer(true)?;
                    for word in words {
                        self.regs.pp_data.set(u32::from_le_bytes(*word));
                    }
                    self.execute(CMD_WRITE)?;
                    rest = rest.get(PP_SIZE..).unwrap_or_default();
                    position = position.wrapping_add(PP_SIZE as u32);
                }
                _ => {
                    self.write_buffer(false)?;
                    self.regs.wdata.set(first);
                    self.execute(CMD_WRITE | CMD_AUTOINC)?;
                    rest = rest.get(1..).unwrap_or_default();
                    position = position.wrapping_add(1);
                }
            }
        }
        self.wait_ready()
    }

    /// Erase `size` bytes at `address`, both 4 KiB aligned: 64 KiB block
    /// erases where aligned, 4 KiB sector erases elsewhere.
    pub fn erase(&mut self, address: u32, size: u32) -> Result<(), Error> {
        check(address, size as usize)?;
        if !address.is_multiple_of(SECTOR) || !size.is_multiple_of(SECTOR) {
            return Err(Error::InvalidArgument);
        }
        let end = address.wrapping_add(size);
        let mut position = address;
        while position < end {
            let (opcode, len, polls) = if position.is_multiple_of(BLOCK) && end - position >= BLOCK
            {
                (OP_BLOCK_ERASE, BLOCK, BLOCK_ERASE_POLLS)
            } else {
                (OP_SECTOR_ERASE, SECTOR, POLLS)
            };
            self.wait_ready()?;
            let regs = self.regs;
            regs.prgdata5.set(OP_WREN);
            regs.cnt.set(8);
            self.execute(CMD_PRG)?;
            let [a0, a1, a2, _] = position.to_le_bytes();
            regs.prgdata5.set(opcode);
            regs.prgdata4.set(a2);
            regs.prgdata3.set(a1);
            regs.prgdata2.set(a0);
            regs.cnt.set(32);
            self.execute(CMD_PRG)?;
            self.wait_ready_polls(polls)?;
            position += len;
        }
        Ok(())
    }
}

/// `len` bytes at `address` lie within the 3-byte address space.
fn check(address: u32, len: usize) -> Result<(), Error> {
    u32::try_from(len)
        .ok()
        .and_then(|len| address.checked_add(len))
        .filter(|&end| end <= MAX_ADDRESS)
        .map(|_| ())
        .ok_or(Error::InvalidArgument)
}

fn poll(polls: u32, mut done: impl FnMut() -> bool) -> Result<(), Error> {
    if (0..polls).any(|_| done()) {
        Ok(())
    } else {
        Err(Error::Timeout)
    }
}

#[cfg(test)]
mod tests {
    use super::{Error, MAX_ADDRESS, check};

    #[test]
    fn requests_stay_in_3byte_address_space() {
        assert_eq!(check(0, 0x100), Ok(()));
        assert_eq!(check(MAX_ADDRESS - 0x10, 0x10), Ok(()));
        assert_eq!(check(MAX_ADDRESS - 0x10, 0x11), Err(Error::InvalidArgument));
        assert_eq!(check(u32::MAX, 1), Err(Error::InvalidArgument));
    }
}
