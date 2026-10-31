//! Chrome OS Embedded Controller: host command protocol v3 over SPI.
//!
//! The platform supplies the SPI controller and chip select through
//! [`EcSpi`]; this module frames requests, waits for the EC's framing byte
//! and validates responses as Linux's cros_ec_spi does.

pub mod keyboard;

use core::fmt::Debug;

use crate::time::Timeout;

/// Byte the EC sends right before a protocol v3 response.
const EC_FRAMING_BYTE: u8 = 0xec;
const EC_CMD_HELLO: u16 = 0x01;
const EC_CMD_GET_CMD_VERSIONS: u16 = 0x08;
/// Request/response packet limit, header included.
const MAX_PACKET: usize = 268;
const HEADER: usize = 8;
/// Minimum chip select deassertion time between two commands.
const RECOVERY_US: u64 = 200;

/// SPI bus with the EC's chip select, as one controller transaction.
pub trait EcSpi {
    type Error: Debug;

    /// Assert chip select and start a transaction that spans all
    /// following [`EcSpi::write`] and [`EcSpi::read`] calls.
    fn begin(&mut self);
    /// Clock out `data`, discarding what the EC sends meanwhile.
    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    /// Clock in `data.len()` bytes.
    fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error>;
    /// End the transaction and deassert chip select; also after errors.
    fn end(&mut self);
}

#[derive(Debug, PartialEq, Eq)]
#[allow(dead_code)] // fields are reported through Debug
pub enum EcError<E> {
    Bus(E),
    FramingTimeout,
    EcStatus(u8),
    Checksum,
    Protocol,
    /// Non-zero `EC_RES_*` result code.
    Result(u16),
}

fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |sum, b| sum.wrapping_add(*b))
}

pub struct CrosEc<S> {
    spi: S,
    cooldown: Timeout,
}

impl<S: EcSpi> CrosEc<S> {
    pub fn new(spi: S) -> Self {
        Self {
            spi,
            cooldown: Timeout::from_us(0),
        }
    }

    /// Protocol v3 host command. Returns the response payload length;
    /// payload beyond `response.len()` is discarded.
    pub fn command(
        &mut self,
        command: u16,
        version: u8,
        params: &[u8],
        response: &mut [u8],
    ) -> Result<usize, EcError<S::Error>> {
        let mut request = [0u8; MAX_PACKET];
        let len = HEADER + params.len();
        if len > MAX_PACKET || HEADER + response.len() > MAX_PACKET {
            return Err(EcError::Protocol);
        }
        request[0] = 3;
        request[2..4].copy_from_slice(&command.to_le_bytes());
        request[4] = version;
        request[6..8].copy_from_slice(&(params.len() as u16).to_le_bytes());
        request[HEADER..len].copy_from_slice(params);
        request[1] = 0u8.wrapping_sub(checksum(&request[..len]));

        while !self.cooldown.is_expired() {}
        self.spi.begin();
        let result = Self::exchange(&mut self.spi, &request[..len], response);
        self.spi.end();
        self.cooldown = Timeout::from_us(RECOVERY_US);
        result
    }

    fn exchange(
        spi: &mut S,
        request: &[u8],
        response: &mut [u8],
    ) -> Result<usize, EcError<S::Error>> {
        spi.write(request).map_err(EcError::Bus)?;
        let deadline = Timeout::from_ms(1000);
        loop {
            let mut byte = [0u8];
            spi.read(&mut byte).map_err(EcError::Bus)?;
            match byte[0] {
                EC_FRAMING_BYTE => break,
                // Receiving/processing/not-ready: keep clocking.
                0xf9..=0xfb | 0x00 | 0xff => {}
                status if status >= 0xf0 => return Err(EcError::EcStatus(status)),
                _ => {}
            }
            if deadline.is_expired() {
                return Err(EcError::FramingTimeout);
            }
        }
        let mut header = [0u8; HEADER];
        spi.read(&mut header).map_err(EcError::Bus)?;
        let data_len = usize::from(u16::from_le_bytes([header[4], header[5]]));
        if header[0] != 3 || data_len > MAX_PACKET - HEADER {
            return Err(EcError::Protocol);
        }
        let mut data = [0u8; MAX_PACKET - HEADER];
        spi.read(&mut data[..data_len]).map_err(EcError::Bus)?;
        if checksum(&header).wrapping_add(checksum(&data[..data_len])) != 0 {
            return Err(EcError::Checksum);
        }
        let result = u16::from_le_bytes([header[2], header[3]]);
        if result != 0 {
            return Err(EcError::Result(result));
        }
        let copied = data_len.min(response.len());
        response[..copied].copy_from_slice(&data[..copied]);
        Ok(copied)
    }

    /// `EC_CMD_HELLO`: true if the EC answered with the expected value.
    pub fn hello(&mut self) -> Result<bool, EcError<S::Error>> {
        let mut out = [0u8; 4];
        let len = self.command(EC_CMD_HELLO, 0, &0xa0b0_c0d0u32.to_le_bytes(), &mut out)?;
        Ok(len == 4 && u32::from_le_bytes(out) == 0xa1b2_c3d4)
    }

    /// Bit mask of supported versions of `command` (`EC_CMD_GET_CMD_VERSIONS`).
    pub fn command_versions(&mut self, command: u8) -> Result<u32, EcError<S::Error>> {
        let mut mask = [0u8; 4];
        match self.command(EC_CMD_GET_CMD_VERSIONS, 0, &[command], &mut mask)? {
            4 => Ok(u32::from_le_bytes(mask)),
            _ => Err(EcError::Protocol),
        }
    }
}

#[cfg(test)]
mod tests;
