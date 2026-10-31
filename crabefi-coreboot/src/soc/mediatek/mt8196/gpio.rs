//! MT8196 GPIO block (coreboot soc/mediatek/common/gpio.c layout): per bank
//! of 32 pins a value/set/reset register tuple, 16 bytes apart, for
//! direction at 0x000, output at 0x100 and input at 0x200; 4-bit pin modes
//! at 0x300, eight pins per tuple.

const GPIO: u64 = 0x1002_d000;
const DIR: u64 = 0x000;
const DIN: u64 = 0x200;
const MODE: u64 = 0x300;
const RST: u64 = 0x8;

fn bank(base: u64, pin: u32) -> (u64, u32) {
    (GPIO + base + u64::from(pin / 32) * 0x10, 1 << (pin % 32))
}

fn read(address: u64) -> u32 {
    // SAFETY: always-on GPIO block registers, Device-mapped.
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

fn write(address: u64, value: u32) {
    // SAFETY: as in `read`; the set/reset registers touch only `value`'s bits.
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

fn mode(pin: u32) -> (u64, u32) {
    (GPIO + MODE + u64::from(pin / 8) * 0x10, (pin % 8) * 4)
}

pub fn set_mode(pin: u32, function: u32) {
    let (address, shift) = mode(pin);
    write(
        address,
        (read(address) & !(0xf << shift)) | ((function & 0xf) << shift),
    );
}

/// GPIO function, input direction.
pub fn make_input(pin: u32) {
    set_mode(pin, 0);
    let (dir, bit) = bank(DIR, pin);
    write(dir + RST, bit);
}

/// Level on the pad.
pub fn input(pin: u32) -> bool {
    let (din, bit) = bank(DIN, pin);
    read(din) & bit != 0
}
