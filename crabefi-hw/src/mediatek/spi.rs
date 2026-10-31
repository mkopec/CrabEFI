// SPDX-License-Identifier: MIT OR Apache-2.0
//! MediaTek SPI master in FIFO pause mode, polled.
//!
//! Port of coreboot's soc/mediatek/common/spi.c transfers on a controller
//! whose pads and timing firmware already set up. Between
//! [`MtkSpi::begin`] and [`MtkSpi::end`] every [`SpiBus`] transfer continues
//! one paused transaction in FIFO-sized chunks, so data-dependent exchanges
//! (wait for a framing byte, then read a length) stay in one chip select
//! assertion. Chip select itself is the platform's (often a GPIO).

use embedded_hal::{
    delay::DelayNs,
    spi::{self, ErrorKind, ErrorType, SpiBus},
};
use tock_registers::{
    interfaces::{ReadWriteable, Readable, Writeable},
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite},
};

register_bitfields![u32,
    Cfg1 [
        /// CS idle timing, kept as configured.
        CS_IDLE OFFSET(0) NUMBITS(8) [],
        PACKET_LENGTH OFFSET(16) NUMBITS(10) []
    ],
    Cmd [
        ACT OFFSET(0) NUMBITS(1) [],
        RESUME OFFSET(1) NUMBITS(1) [],
        RST OFFSET(2) NUMBITS(1) [],
        PAUSE_EN OFFSET(4) NUMBITS(1) []
    ],
    Status0 [
        /// Finished or paused.
        DONE OFFSET(0) NUMBITS(2) []
    ],
    Status1 [BUSY_DONE OFFSET(0) NUMBITS(1) []]
];

register_structs! {
    Regs {
        (0x00 => cfg0: ReadOnly<u32>),
        (0x04 => cfg1: ReadWrite<u32, Cfg1::Register>),
        (0x08 => _reserved0),
        (0x10 => tx_data: ReadWrite<u32>),
        (0x14 => rx_data: ReadOnly<u32>),
        (0x18 => cmd: ReadWrite<u32, Cmd::Register>),
        (0x1c => status0: ReadOnly<u32, Status0::Register>),
        (0x20 => status1: ReadOnly<u32, Status1::Register>),
        (0x24 => @END),
    }
}

const FIFO_DEPTH: usize = 32;
/// Filler clocked out while only reading.
const IDLE_WORD: u32 = 0xdead_dead;
const CHUNK_TIMEOUT_US: u32 = 1_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidRegion,
    Timeout,
}

impl spi::Error for Error {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub struct MtkSpi<D> {
    regs: &'static Regs,
    delay: D,
    started: bool,
}

impl<D: DelayNs> MtkSpi<D> {
    /// # Safety
    /// `base` is the controller's register block, clocked and Device-mapped,
    /// used by nothing else while this driver exists.
    pub unsafe fn new(base: usize, delay: D) -> Result<Self, Error> {
        if base == 0 || base & 3 != 0 {
            return Err(Error::InvalidRegion);
        }
        Ok(Self {
            // SAFETY: the caller's hardware contract.
            regs: unsafe { &*(base as *const Regs) },
            delay,
            started: false,
        })
    }

    /// Whether firmware programmed the bus timing (CFG0).
    pub fn is_configured(&self) -> bool {
        self.regs.cfg0.get() != 0
    }

    /// Reset the controller state machine.
    pub fn reset(&mut self) {
        self.regs.cmd.modify(Cmd::RST::SET);
        self.regs.cmd.modify(Cmd::RST::CLEAR);
    }

    /// Start a paused transaction; assert chip select afterwards.
    pub fn begin(&mut self) {
        self.regs.cmd.modify(Cmd::PAUSE_EN::SET);
        self.started = false;
    }

    /// End the transaction; deassert chip select afterwards.
    pub fn end(&mut self) {
        self.regs.cmd.modify(Cmd::PAUSE_EN::CLEAR);
        self.reset();
    }

    /// One FIFO-sized full-duplex chunk, as coreboot's do_transfer().
    fn chunk(
        &mut self,
        out: Option<&[u8]>,
        input: Option<&mut [u8]>,
        len: usize,
    ) -> Result<(), Error> {
        let regs = self.regs;
        let cs_idle = regs.cfg1.read(Cfg1::CS_IDLE);
        regs.cfg1
            .write(Cfg1::CS_IDLE.val(cs_idle) + Cfg1::PACKET_LENGTH.val(len as u32 - 1));
        for word in 0..len.div_ceil(4) {
            let value = out.map_or(IDLE_WORD, |out| {
                (0..4).fold(0, |value, byte| {
                    let b = out.get(word * 4 + byte).copied().unwrap_or(0);
                    value | u32::from(b) << (byte * 8)
                })
            });
            regs.tx_data.set(value);
        }
        regs.cmd.modify(if self.started {
            Cmd::RESUME::SET
        } else {
            Cmd::ACT::SET
        });
        self.started = true;
        let mut budget = CHUNK_TIMEOUT_US;
        while !regs.status1.is_set(Status1::BUSY_DONE) {
            budget = budget.checked_sub(1).ok_or(Error::Timeout)?;
            self.delay.delay_us(1);
        }
        while regs.status0.read(Status0::DONE) == 0 {
            budget = budget.checked_sub(1).ok_or(Error::Timeout)?;
            self.delay.delay_us(1);
        }
        if let Some(input) = input {
            let mut word = 0;
            for (index, byte) in input.iter_mut().take(len).enumerate() {
                if index % 4 == 0 {
                    word = regs.rx_data.get();
                }
                *byte = (word >> ((index % 4) * 8)) as u8;
            }
        }
        Ok(())
    }
}

impl<D> ErrorType for MtkSpi<D> {
    type Error = Error;
}

impl<D: DelayNs> SpiBus for MtkSpi<D> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Error> {
        for piece in words.chunks_mut(FIFO_DEPTH) {
            let len = piece.len();
            self.chunk(None, Some(piece), len)?;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Error> {
        for piece in words.chunks(FIFO_DEPTH) {
            self.chunk(Some(piece), None, piece.len())?;
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error> {
        let len = read.len().max(write.len());
        let mut offset = 0;
        while offset < len {
            let n = (len - offset).min(FIFO_DEPTH);
            let out = write.get(offset..).unwrap_or(&[]);
            let input = read.get_mut(offset..).unwrap_or(&mut []);
            self.chunk(Some(out), Some(input), n)?;
            offset += n;
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Error> {
        let mut out = [0u8; FIFO_DEPTH];
        for piece in words.chunks_mut(FIFO_DEPTH) {
            let len = piece.len();
            out[..len].copy_from_slice(piece);
            self.chunk(Some(&out[..len]), Some(piece), len)?;
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Error> {
        Ok(())
    }
}
