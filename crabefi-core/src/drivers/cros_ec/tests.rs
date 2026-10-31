use alloc::collections::VecDeque;
use alloc::vec::Vec;

use super::{CrosEc, EcError, EcSpi, checksum};

#[derive(Default)]
struct MockSpi {
    written: Vec<u8>,
    replies: VecDeque<u8>,
    selected: bool,
    transactions: usize,
}

impl EcSpi for MockSpi {
    type Error = ();

    fn begin(&mut self) {
        assert!(!self.selected);
        self.selected = true;
    }

    fn write(&mut self, data: &[u8]) -> Result<(), ()> {
        assert!(self.selected);
        self.written.extend_from_slice(data);
        Ok(())
    }

    fn read(&mut self, data: &mut [u8]) -> Result<(), ()> {
        assert!(self.selected);
        for byte in data {
            *byte = self.replies.pop_front().ok_or(())?;
        }
        Ok(())
    }

    fn end(&mut self) {
        self.selected = false;
        self.transactions += 1;
    }
}

/// Not-ready filler, framing byte, then a v3 response with `result` and `data`.
fn reply(result: u16, data: &[u8]) -> VecDeque<u8> {
    let mut header = [3u8, 0, 0, 0, 0, 0, 0, 0];
    header[2..4].copy_from_slice(&result.to_le_bytes());
    header[4..6].copy_from_slice(&(data.len() as u16).to_le_bytes());
    header[1] = 0u8.wrapping_sub(checksum(&header).wrapping_add(checksum(data)));
    let mut bytes = VecDeque::from([0xfb, 0xfa, 0xec]);
    bytes.extend(header);
    bytes.extend(data);
    bytes
}

#[test]
fn hello_frames_request_and_parses_response() {
    let spi = MockSpi {
        replies: reply(0, &0xa1b2_c3d4u32.to_le_bytes()),
        ..Default::default()
    };
    let mut ec = CrosEc::new(spi);
    assert_eq!(ec.hello(), Ok(true));
    let request = &ec.spi.written;
    assert_eq!(request.len(), 12);
    assert_eq!(&request[..1], &[3]);
    assert_eq!(&request[2..8], &[0x01, 0, 0, 0, 4, 0]);
    assert_eq!(checksum(request), 0);
    assert!(!ec.spi.selected);
    assert_eq!(ec.spi.transactions, 1);
}

#[test]
fn rejects_bad_checksum_and_reports_result_codes() {
    let mut spi = MockSpi {
        replies: reply(0, &[1, 2, 3, 4]),
        ..Default::default()
    };
    let last = spi.replies.len() - 1;
    spi.replies[last] ^= 0xff;
    let mut ec = CrosEc::new(spi);
    assert_eq!(ec.command_versions(0x67), Err(EcError::Checksum));
    assert!(!ec.spi.selected);

    ec.spi.replies = reply(9, &[]);
    assert_eq!(
        ec.command(0x67, 3, &[], &mut [0; 8]),
        Err(EcError::Result(9))
    );
}

#[test]
fn ec_status_byte_aborts_and_deselects() {
    let spi = MockSpi {
        replies: VecDeque::from([0xfe]),
        ..Default::default()
    };
    let mut ec = CrosEc::new(spi);
    assert_eq!(ec.hello(), Err(EcError::EcStatus(0xfe)));
    assert!(!ec.spi.selected);
}

#[test]
fn truncates_payload_to_response_buffer() {
    let spi = MockSpi {
        replies: reply(0, &[1, 2, 3, 4, 5, 6]),
        ..Default::default()
    };
    let mut ec = CrosEc::new(spi);
    let mut out = [0u8; 4];
    assert_eq!(ec.command(0x67, 3, &[], &mut out), Ok(4));
    assert_eq!(out, [1, 2, 3, 4]);
}
