//! Chrome EC keyboard: MKBP key-matrix events translated through a
//! devicetree `linux,keymap`.

use alloc::boxed::Box;
use alloc::vec::Vec;

use super::{CrosEc, EcError, EcSpi};
use crate::drivers::keyboard_common::{self, KeyboardDevice, efi_shift_state};
use crate::time::Timeout;
use crate::{Key, KeyState};

const EC_CMD_GET_NEXT_EVENT: u16 = 0x67;
const EC_RES_UNAVAILABLE: u16 = 9;
const MKBP_EVENT_KEY_MATRIX: u8 = 0;
/// Matrix columns of `GET_NEXT_EVENT` v3.
const COLUMNS: usize = 18;
const POLL_INTERVAL_US: u64 = 15_000;

/// Linux input keycode -> (EFI scan code, unshifted char, shifted char).
fn translate(code: u16) -> (u16, u8, u8) {
    const ROW1: &[u8; 13] = b"1234567890-=\x08";
    const ROW1S: &[u8; 13] = b"!@#$%^&*()_+\x08";
    const ROW2: &[u8; 14] = b"\tqwertyuiop[]\r";
    const ROW2S: &[u8; 14] = b"\tQWERTYUIOP{}\r";
    const ROW3: &[u8; 12] = b"asdfghjkl;'`";
    const ROW3S: &[u8; 12] = b"ASDFGHJKL:\"~";
    const ROW4: &[u8; 11] = b"\\zxcvbnm,./";
    const ROW4S: &[u8; 11] = b"|ZXCVBNM<>?";
    let c = usize::from(code);
    match code {
        1 => (0x17, 0, 0),
        2..=14 => (0, ROW1[c - 2], ROW1S[c - 2]),
        15..=28 => (0, ROW2[c - 15], ROW2S[c - 15]),
        30..=41 => (0, ROW3[c - 30], ROW3S[c - 30]),
        43..=53 => (0, ROW4[c - 43], ROW4S[c - 43]),
        57 => (0, b' ', b' '),
        59..=68 => (0x0b + code - 59, 0, 0), // F1..F10
        87 => (0x15, 0, 0),
        88 => (0x16, 0, 0),
        102 => (0x05, 0, 0),
        103 => (0x01, 0, 0),
        104 => (0x09, 0, 0),
        105 => (0x04, 0, 0),
        106 => (0x03, 0, 0),
        107 => (0x06, 0, 0),
        108 => (0x02, 0, 0),
        109 => (0x0a, 0, 0),
        110 => (0x07, 0, 0),
        111 => (0x08, 0, 0),
        _ => (0, 0, 0),
    }
}

/// Linux keycode at a matrix position, from a `linux,keymap`.
fn keycode(keymap: &[u32], row: usize, col: usize) -> Option<u16> {
    keymap
        .iter()
        .find(|&&entry| (entry >> 24) as usize == row && ((entry >> 16) & 0xff) as usize == col)
        .map(|&entry| entry as u16)
}

struct EcKeyboard<S> {
    ec: CrosEc<S>,
    version: u8,
    state: [u8; COLUMNS],
    queue: heapless::Deque<Key, 16>,
    shift: u32,
    next_poll: Timeout,
    failures: u32,
    keymap: Vec<u32>,
}

impl<S: EcSpi> EcKeyboard<S> {
    /// Next key-matrix MKBP event, if any. Other event types are discarded.
    fn next_matrix(&mut self) -> Result<Option<[u8; COLUMNS]>, EcError<S::Error>> {
        for _ in 0..8 {
            let mut event = [0u8; 64];
            match self
                .ec
                .command(EC_CMD_GET_NEXT_EVENT, self.version, &[], &mut event)
            {
                Ok(0) | Err(EcError::Result(EC_RES_UNAVAILABLE)) => return Ok(None),
                Ok(len) if event[0] & 0x7f == MKBP_EVENT_KEY_MATRIX => {
                    let mut matrix = [0u8; COLUMNS];
                    let n = (len - 1).min(COLUMNS);
                    matrix[..n].copy_from_slice(&event[1..1 + n]);
                    return Ok(Some(matrix));
                }
                Ok(_) => continue, // host/sensor/switch/... event: not ours
                Err(error) => return Err(error),
            }
        }
        Ok(None)
    }

    fn poll(&mut self) {
        if !self.next_poll.is_expired() || self.failures > 20 {
            return;
        }
        self.next_poll = Timeout::from_us(POLL_INTERVAL_US);
        let matrix = match self.next_matrix() {
            Ok(Some(matrix)) => {
                self.failures = 0;
                matrix
            }
            Ok(None) => {
                self.failures = 0;
                return;
            }
            Err(error) => {
                self.failures += 1;
                if self.failures == 1 || self.failures > 20 {
                    log::warn!("EC keyboard: matrix read failed: {error:?}");
                }
                return;
            }
        };
        self.apply(matrix);
    }

    /// Queue keys newly pressed in `matrix` and update the modifier state.
    fn apply(&mut self, matrix: [u8; COLUMNS]) {
        let mut shift = 0;
        for (col, &bits) in matrix.iter().enumerate() {
            for row in 0..8 {
                let pressed = bits & (1 << row) != 0;
                let was = self.state[col] & (1 << row) != 0;
                let Some(code) = keycode(&self.keymap, row, col) else {
                    continue;
                };
                if pressed {
                    shift |= match code {
                        42 => efi_shift_state::LEFT_SHIFT_PRESSED,
                        54 => efi_shift_state::RIGHT_SHIFT_PRESSED,
                        29 => efi_shift_state::LEFT_CONTROL_PRESSED,
                        97 => efi_shift_state::RIGHT_CONTROL_PRESSED,
                        56 => efi_shift_state::LEFT_ALT_PRESSED,
                        100 => efi_shift_state::RIGHT_ALT_PRESSED,
                        125 => efi_shift_state::LEFT_LOGO_PRESSED,
                        _ => 0,
                    };
                }
                if pressed && !was {
                    let shifted = (self.shift | shift)
                        & (efi_shift_state::LEFT_SHIFT_PRESSED
                            | efi_shift_state::RIGHT_SHIFT_PRESSED)
                        != 0;
                    let (scancode, plain, upper) = translate(code);
                    let unicode_char = u16::from(if shifted { upper } else { plain });
                    if scancode != 0 || unicode_char != 0 {
                        let _ = self.queue.push_back(Key {
                            scancode,
                            unicode_char,
                        });
                    }
                }
            }
        }
        self.shift = shift;
        self.state = matrix;
    }
}

impl<S: EcSpi> KeyboardDevice for EcKeyboard<S> {
    fn read_key(&mut self) -> Option<Key> {
        self.poll();
        self.queue.pop_front()
    }

    fn key_state(&mut self) -> KeyState {
        KeyState {
            shift_state: self.shift,
            toggle_state: 0,
        }
    }
}

/// Register the EC's keyboard, with matrix positions mapped by `keymap`
/// (`linux,keymap` entries). Returns false if the EC does not answer or has
/// no MKBP event support.
pub fn register<S: EcSpi + 'static>(mut ec: CrosEc<S>, keymap: Vec<u32>) -> bool {
    match ec.hello() {
        Ok(true) => log::info!("EC keyboard: EC hello OK"),
        result => {
            log::error!("EC keyboard: EC hello failed: {result:?}");
            return false;
        }
    }
    // Use the newest GET_NEXT_EVENT version (v3 carries 18 matrix columns).
    let version = match ec.command_versions(EC_CMD_GET_NEXT_EVENT as u8) {
        Ok(mask) if mask != 0 => (31 - mask.leading_zeros()).min(3) as u8,
        result => {
            log::error!("EC keyboard: no MKBP event support: {result:?}");
            return false;
        }
    };
    let mut keyboard = EcKeyboard {
        ec,
        version,
        state: [0; COLUMNS],
        queue: heapless::Deque::new(),
        shift: 0,
        next_poll: Timeout::from_us(0),
        failures: 0,
        keymap,
    };
    // Drain events queued before the payload; keep the latest matrix as the
    // baseline so keys held at boot are not reported as new presses.
    for _ in 0..32 {
        match keyboard.next_matrix() {
            Ok(Some(matrix)) => keyboard.state = matrix,
            Ok(None) => break,
            Err(error) => {
                log::error!("EC keyboard: GET_NEXT_EVENT v{version} failed: {error:?}");
                return false;
            }
        }
    }
    log::info!("EC keyboard: registered (GET_NEXT_EVENT v{version})");
    keyboard_common::register(Box::new(keyboard));
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keymap_lookup_and_translation() {
        // KEY(row 1, col 2, KEY_A), KEY(row 0, col 0, KEY_LEFTSHIFT)
        let keymap = [0x0102_001e, 0x0000_002a];
        assert_eq!(keycode(&keymap, 1, 2), Some(30));
        assert_eq!(keycode(&keymap, 0, 0), Some(42));
        assert_eq!(keycode(&keymap, 2, 1), None);
        assert_eq!(translate(30), (0, b'a', b'A'));
        assert_eq!(translate(1), (0x17, 0, 0));
        assert_eq!(translate(68), (0x14, 0, 0));
    }
}
