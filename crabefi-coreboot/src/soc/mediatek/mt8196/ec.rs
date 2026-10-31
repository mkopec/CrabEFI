//! Chrome EC SPI bus of MT8196 Chromebooks, for the core EC keyboard.
//!
//! coreboot's bootblock initializes SPI1 (pinmux, ~3 MHz timing) and drives
//! its chip select (GPIO111) high as a GPIO output. This driver reuses that
//! setup, as coreboot's own ramstage EC commands do, and only toggles CS.

use crabefi::drivers::cros_ec::{CrosEc, EcSpi, keyboard};
use crabefi::time::{Delay, delay_us};
use crabefi_hw::mediatek::spi::{Error, MtkSpi};
use embedded_hal::spi::SpiBus;

use super::gpio;

/// SPI1 chip select pad, a GPIO output since coreboot's bootblock.
const CS_PIN: u32 = 111;

/// Chip select is active low.
fn chip_select(active: bool) {
    gpio::set_output(CS_PIN, !active);
}

/// SPI1 with the EC's GPIO chip select.
struct EcBus(MtkSpi<Delay>);

impl EcSpi for EcBus {
    type Error = Error;

    fn begin(&mut self) {
        self.0.begin();
        chip_select(true);
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Error> {
        self.0.write(data)
    }

    fn read(&mut self, data: &mut [u8]) -> Result<(), Error> {
        self.0.read(data)
    }

    fn end(&mut self) {
        self.0.end();
        chip_select(false);
    }
}

pub fn init() {
    let board = super::dt::board();
    let Some(base) = board.ec_spi else {
        log::info!("EC keyboard: no EC in the board description");
        return;
    };
    if !gpio::is_gpio_output(CS_PIN) {
        log::error!("EC keyboard: SPI1 chip select GPIO111 is not a GPIO output; skipping");
        return;
    }
    // SAFETY: the board description's `google,cros-ec-spi` controller,
    // clocked and Device-mapped by coreboot, used only here.
    let mut spi = match unsafe { MtkSpi::new(base as usize, Delay) } {
        Ok(spi) => spi,
        Err(error) => {
            log::error!("EC keyboard: SPI {base:#x}: {error:?}");
            return;
        }
    };
    if !spi.is_configured() {
        log::error!("EC keyboard: SPI1 timing not initialized by coreboot; skipping");
        return;
    }
    spi.reset();
    chip_select(false);
    delay_us(200);
    keyboard::register(CrosEc::new(EcBus(spi)), board.ec_keymap);
}
