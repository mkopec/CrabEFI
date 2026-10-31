//! HID over I2C (ACPI `PNP0C50`): touchpads, mice and keyboards.
//!
//! The device's report descriptor decides what it is. Mouse collections
//! drive the pointer, which also covers precision touchpads in their power-on
//! mouse mode, and keyboard collections drive the keyboard through the same
//! key handling as USB boot keyboards. Precision touchpads without a mouse
//! collection are switched to touchpad input mode, as Linux's hid-multitouch
//! does, and their primary contact drives the pointer. Touchscreens are not
//! interpreted.

use super::keyboard_common::KeyboardDevice;
use super::pointer::{PointerDevice, PointerReport};
use super::usb::hid_keyboard::{BootKeys, KeyboardReport};
use crate::peripheral::{I2cBus, I2cPeripheral, Interrupt};
use crate::platform::{Key, KeyState};
use crate::time::{Timeout, delay_ms};
use alloc::{boxed::Box, rc::Rc, vec, vec::Vec};
use core::cell::RefCell;
use embedded_hal::i2c::I2c;
use hidreport::{Field, FieldAttributes, Report, ReportDescriptor, Usage, VariableField};
use zerocopy::{FromBytes, Immutable, KnownLayout, little_endian::U16};

/// ACPI identifiers of HID-over-I2C devices.
pub(crate) const IDS: &[&str] = &["PNP0C50", "ACPI0C50"];

const OPCODE_RESET: u8 = 0x01;
const OPCODE_SET_REPORT: u8 = 0x03;
const OPCODE_SET_POWER: u8 = 0x08;
const REPORT_TYPE_FEATURE: u8 = 0x03;
const POWER_ON: u8 = 0x00;
const POWER_SLEEP: u8 = 0x01;
/// Bound the work of one poll.
const REPORTS_PER_POLL: usize = 8;
const MAX_REPORT_DESCRIPTOR: usize = 4096;
const MAX_INPUT: usize = 512;

const GENERIC_DESKTOP: u16 = 0x01;
const MOUSE: u16 = 0x02;
const KEYBOARD: u16 = 0x06;
const X: u16 = 0x30;
const Y: u16 = 0x31;
const WHEEL: u16 = 0x38;
const KEY_CODES: u16 = 0x07;
const BUTTONS: u16 = 0x09;
const DIGITIZER: u16 = 0x0d;
const TOUCHPAD: u16 = 0x05;
const TIP_SWITCH: u16 = 0x42;
const CONTACT_ID: u16 = 0x51;
const CONTACT_COUNT: u16 = 0x54;
const INPUT_MODE: u16 = 0x52;
const SURFACE_SWITCH: u16 = 0x57;
const BUTTON_SWITCH: u16 = 0x58;
/// hid-multitouch MT_INPUTMODE_TOUCHPAD.
const INPUT_MODE_TOUCHPAD: u32 = 3;

/// HID descriptor, HID over I2C specification section 5.1.
#[repr(C)]
#[derive(FromBytes, KnownLayout, Immutable)]
struct Descriptor {
    length: U16,
    version: U16,
    report_descriptor_length: U16,
    report_descriptor_register: U16,
    input_register: U16,
    max_input_length: U16,
    output_register: U16,
    max_output_length: U16,
    command_register: U16,
    data_register: U16,
    vendor: U16,
    product: U16,
    firmware: U16,
    reserved: [u8; 4],
}

#[derive(Debug)]
pub enum Error<E> {
    Bus(E),
    NoInterrupt,
    NoDescriptorRegister,
    UnsupportedDescriptor,
    ResetAcknowledgement,
    ReportDescriptor,
    NoUsableCollection,
    InvalidReport,
}

fn usage(page: u16, id: u16) -> u32 {
    (u32::from(page) << 16) | u32::from(id)
}

fn in_collection(field: &VariableField, collection: u32) -> bool {
    field
        .collections
        .iter()
        .any(|c| c.usages().iter().any(|u| u32::from(u) == collection))
}

fn variables(report: &impl Report, collection: u32) -> impl Iterator<Item = &VariableField> + '_ {
    report.fields().iter().filter_map(move |field| match field {
        Field::Variable(field) if in_collection(field, collection) => Some(field),
        _ => None,
    })
}

fn variable(report: &impl Report, collection: u32, usage: u32) -> Option<VariableField> {
    variables(report, collection)
        .find(|field| u32::from(&field.usage) == usage)
        .cloned()
}

/// Relative mouse fields of one input report.
struct Mouse {
    report_id: Option<u8>,
    size: usize,
    x: VariableField,
    y: VariableField,
    wheel: Option<VariableField>,
    buttons: Vec<(u32, VariableField)>,
}

impl Mouse {
    fn find(descriptor: &ReportDescriptor) -> Option<Self> {
        let collection = usage(GENERIC_DESKTOP, MOUSE);
        descriptor.input_reports().iter().find_map(|report| {
            let x = variable(report, collection, usage(GENERIC_DESKTOP, X))?;
            let y = variable(report, collection, usage(GENERIC_DESKTOP, Y))?;
            if !x.is_relative() || !y.is_relative() {
                return None;
            }
            let buttons = variables(report, collection)
                .filter_map(|field| {
                    let Usage {
                        usage_page,
                        usage_id,
                    } = field.usage;
                    let index = u16::from(usage_id);
                    (u16::from(usage_page) == BUTTONS && (1..=3).contains(&index))
                        .then(|| (1 << (index - 1), field.clone()))
                })
                .collect();
            Some(Self {
                report_id: (*report.report_id()).map(u8::from),
                size: report.size_in_bytes(),
                x,
                y,
                wheel: variable(report, collection, usage(GENERIC_DESKTOP, WHEEL)),
                buttons,
            })
        })
    }

    fn read(&self, report: &[u8]) -> Option<PointerReport> {
        let value = |field: &VariableField| field.extract(report).ok().map(i32::from);
        Some(PointerReport {
            dx: value(&self.x)?,
            dy: value(&self.y)?,
            // HID wheel up is positive; EFI and the UI scroll down positive.
            dz: self.wheel.as_ref().and_then(value).map_or(0, |dz| -dz),
            buttons: self
                .buttons
                .iter()
                .filter(|(_, field)| value(field).is_some_and(|pressed| pressed != 0))
                .fold(0, |buttons, (bit, _)| buttons | bit),
        })
    }
}

/// One contact slot of a precision touchpad report.
struct Slot {
    tip: VariableField,
    id: Option<VariableField>,
    x: VariableField,
    y: VariableField,
}

#[derive(Clone, Copy)]
struct Contact {
    id: i32,
    x: i32,
    y: i32,
}

/// Precision touchpad (Windows PTP) input report fields.
struct Touchpad {
    report_id: Option<u8>,
    size: usize,
    slots: Vec<Slot>,
    count: Option<VariableField>,
    button: Option<VariableField>,
}

impl Touchpad {
    fn find(descriptor: &ReportDescriptor) -> Option<Self> {
        let collection = usage(DIGITIZER, TOUCHPAD);
        descriptor.input_reports().iter().find_map(|report| {
            let all = |page, id| {
                variables(report, collection)
                    .filter(move |field| u32::from(&field.usage) == usage(page, id))
                    .cloned()
                    .collect::<Vec<_>>()
            };
            let (xs, ys, tips) = (
                all(GENERIC_DESKTOP, X),
                all(GENERIC_DESKTOP, Y),
                all(DIGITIZER, TIP_SWITCH),
            );
            let mut ids = all(DIGITIZER, CONTACT_ID).into_iter();
            let slots: Vec<_> = xs
                .into_iter()
                .zip(ys)
                .zip(tips)
                .filter(|((x, y), _)| !x.is_relative() && !y.is_relative())
                .map(|((x, y), tip)| Slot {
                    tip,
                    id: ids.next(),
                    x,
                    y,
                })
                .collect();
            if slots.is_empty() {
                return None;
            }
            Some(Self {
                report_id: (*report.report_id()).map(u8::from),
                size: report.size_in_bytes(),
                slots,
                count: variable(report, collection, usage(DIGITIZER, CONTACT_COUNT)),
                button: variable(report, collection, usage(BUTTONS, 1)),
            })
        })
    }

    /// The primary (first touching) contact, the contact count and the
    /// clickpad button; `None` for hybrid-mode continuation reports.
    fn read(&self, report: &[u8]) -> Option<(Option<Contact>, u32, bool)> {
        let value = |field: &VariableField| field.extract(report).ok().map(i32::from);
        let touching = |slot: &&Slot| value(&slot.tip).is_some_and(|tip| tip != 0);
        let count = match &self.count {
            Some(field) => u32::try_from(value(field)?).ok()?,
            None => self.slots.iter().filter(touching).count() as u32,
        };
        if self.count.is_some() && count == 0 && self.slots.iter().any(|slot| touching(&slot)) {
            return None;
        }
        let contact = self.slots.iter().find(touching).and_then(|slot| {
            Some(Contact {
                id: slot.id.as_ref().and_then(value).unwrap_or(0),
                x: value(&slot.x)?,
                y: value(&slot.y)?,
            })
        });
        let button = self.button.as_ref().and_then(value).is_some_and(|b| b != 0);
        Some((contact, count, button))
    }
}

/// Write `value` into the little-endian bit range `bits` of `buffer`.
fn put_bits(buffer: &mut [u8], bits: &core::ops::Range<usize>, value: u32) {
    for (i, bit) in bits.clone().enumerate() {
        let mask = 1 << (bit % 8);
        if i < 32 && value & (1 << i) != 0 {
            buffer[bit / 8] |= mask;
        } else {
            buffer[bit / 8] &= !mask;
        }
    }
}

/// Boot-keyboard-like fields of one input report.
struct Keyboard {
    report_id: Option<u8>,
    size: usize,
    modifiers: Vec<(u8, VariableField)>,
    keys: hidreport::ArrayField,
}

impl Keyboard {
    fn find(descriptor: &ReportDescriptor) -> Option<Self> {
        let collection = usage(GENERIC_DESKTOP, KEYBOARD);
        descriptor.input_reports().iter().find_map(|report| {
            let keys = report.fields().iter().find_map(|field| match field {
                Field::Array(field)
                    if field
                        .collections
                        .iter()
                        .any(|c| c.usages().iter().any(|u| u32::from(u) == collection))
                        && field
                            .usages()
                            .first()
                            .is_some_and(|u| u16::from(u.usage_page) == KEY_CODES) =>
                {
                    Some(field.clone())
                }
                _ => None,
            })?;
            let modifiers = variables(report, collection)
                .filter_map(|field| {
                    let id = u16::from(field.usage.usage_id);
                    (u16::from(field.usage.usage_page) == KEY_CODES && (0xe0..=0xe7).contains(&id))
                        .then(|| (1 << (id - 0xe0), field.clone()))
                })
                .collect();
            Some(Self {
                report_id: (*report.report_id()).map(u8::from),
                size: report.size_in_bytes(),
                modifiers,
                keys,
            })
        })
    }

    fn read(&self, report: &[u8]) -> Option<KeyboardReport> {
        let modifiers = self
            .modifiers
            .iter()
            .filter(|(_, field)| field.extract(report).is_ok_and(|v| u32::from(v) != 0))
            .fold(0, |modifiers, (bit, _)| modifiers | bit);
        let mut keys = [0; 6];
        self.keys
            .extract(report)
            .ok()?
            .iter()
            .map(|key| u8::try_from(u32::from(key)).unwrap_or(0))
            .filter(|&key| key > 3)
            .take(keys.len())
            .enumerate()
            .for_each(|(slot, key)| keys[slot] = key);
        Some(KeyboardReport {
            modifiers,
            reserved: 0,
            keys,
        })
    }
}

/// Whether `report` is the input report with `id` and at least `size` bytes.
fn matches(report: &[u8], id: Option<u8>, size: usize) -> bool {
    report.len() >= size.max(1) && id.is_none_or(|id| report[0] == id)
}

/// One HID-over-I2C device, shared by its pointer and keyboard functions.
pub struct HidI2c<I> {
    bus: I,
    address: u8,
    interrupt: Interrupt,
    command_register: u16,
    data_register: u16,
    max_input: usize,
    mouse: Option<Mouse>,
    touchpad: Option<Touchpad>,
    previous: Option<Contact>,
    keyboard: Option<Keyboard>,
    /// Coalesce motion only within one button level, never across a click.
    reports: heapless::Deque<PointerReport, 16>,
    buttons: u32,
    keys: BootKeys,
    enabled: bool,
}

impl<I: I2c> HidI2c<I> {
    /// Fetch the HID descriptor at `register`, reset and power on the device
    /// and parse its report descriptor.
    pub fn open(
        mut bus: I,
        address: u8,
        interrupt: Interrupt,
        register: u16,
    ) -> Result<Self, Error<I::Error>> {
        let mut raw = [0; size_of::<Descriptor>()];
        bus.write_read(address, &register.to_le_bytes(), &mut raw)
            .map_err(Error::Bus)?;
        let descriptor =
            Descriptor::read_from_bytes(&raw).map_err(|_| Error::UnsupportedDescriptor)?;
        if descriptor.length.get() != 30
            || descriptor.version.get() != 0x0100
            || !(1..=MAX_REPORT_DESCRIPTOR)
                .contains(&usize::from(descriptor.report_descriptor_length.get()))
            || !(3..=MAX_INPUT).contains(&usize::from(descriptor.max_input_length.get()))
        {
            return Err(Error::UnsupportedDescriptor);
        }
        let report_length = usize::from(descriptor.report_descriptor_length.get());
        let max_input = usize::from(descriptor.max_input_length.get());
        let mut device = Self {
            bus,
            address,
            interrupt,
            command_register: descriptor.command_register.get(),
            data_register: descriptor.data_register.get(),
            max_input,
            mouse: None,
            touchpad: None,
            previous: None,
            keyboard: None,
            reports: heapless::Deque::new(),
            buttons: 0,
            keys: BootKeys::default(),
            enabled: true,
        };

        device.command(OPCODE_SET_POWER, POWER_ON)?;
        // Match Linux's settling time for devices that do not clock-stretch
        // while waking; an immediate RESET fails on several HID controllers.
        delay_ms(60);
        device.command(OPCODE_RESET, 0)?;
        // The device signals the end of a reset with a zero-length report.
        let reset = Timeout::from_ms(1_000);
        while !device.interrupt.is_pending() {
            if reset.is_expired() {
                return Err(Error::ResetAcknowledgement);
            }
            delay_ms(1);
        }
        let mut input = vec![0; max_input];
        device.bus.read(address, &mut input).map_err(Error::Bus)?;
        if input[..2] != [0, 0] {
            return Err(Error::ResetAcknowledgement);
        }

        let mut report_descriptor = vec![0; report_length];
        device
            .bus
            .write_read(
                address,
                &descriptor.report_descriptor_register.get().to_le_bytes(),
                &mut report_descriptor,
            )
            .map_err(Error::Bus)?;
        let report_descriptor = ReportDescriptor::try_from(report_descriptor.as_slice())
            .map_err(|_| Error::ReportDescriptor)?;
        device.mouse = Mouse::find(&report_descriptor);
        device.keyboard = Keyboard::find(&report_descriptor);
        if device.mouse.is_none() {
            device.touchpad = Touchpad::find(&report_descriptor);
        }
        if device.mouse.is_none() && device.keyboard.is_none() && device.touchpad.is_none() {
            return Err(Error::NoUsableCollection);
        }
        if device.touchpad.is_some() {
            device.set_touchpad_mode(&report_descriptor)?;
        }
        log::info!(
            "HID I2C device {:04x}:{:04x} at {:#x}: mouse {}, touchpad {}, keyboard {}",
            descriptor.vendor.get(),
            descriptor.product.get(),
            address,
            device.mouse.is_some(),
            device.touchpad.as_ref().map_or(0, |t| t.slots.len()),
            device.keyboard.is_some()
        );
        Ok(device)
    }

    fn command(&mut self, opcode: u8, argument: u8) -> Result<(), Error<I::Error>> {
        let [low, high] = self.command_register.to_le_bytes();
        self.bus
            .write(self.address, &[low, high, argument, opcode])
            .map_err(Error::Bus)
    }

    /// hid-multitouch mt_set_modes(): Input Mode = touchpad, and surface and
    /// button switches on, in every feature report that carries them.
    fn set_touchpad_mode(&mut self, descriptor: &ReportDescriptor) -> Result<(), Error<I::Error>> {
        for report in descriptor.feature_reports() {
            let mut buffer = vec![0u8; report.size_in_bytes()];
            let mut used = false;
            for field in report.fields() {
                let Field::Variable(field) = field else {
                    continue;
                };
                let value = match u32::from(&field.usage) {
                    u if u == usage(DIGITIZER, INPUT_MODE) => INPUT_MODE_TOUCHPAD,
                    u if u == usage(DIGITIZER, SURFACE_SWITCH) => 1,
                    u if u == usage(DIGITIZER, BUTTON_SWITCH) => 1,
                    _ => continue,
                };
                put_bits(&mut buffer, &field.bits, value);
                used = true;
            }
            if used {
                let id = (*report.report_id()).map_or(0, u8::from);
                if id != 0 {
                    buffer[0] = id;
                }
                log::debug!(
                    "HID I2C {:#x}: SET_REPORT feature {id} {buffer:02x?}",
                    self.address
                );
                self.set_feature(id, &buffer)?;
            }
        }
        Ok(())
    }

    /// i2c-hid SET_REPORT; `report` starts with the report ID when non-zero.
    fn set_feature(&mut self, id: u8, report: &[u8]) -> Result<(), Error<I::Error>> {
        let mut command = Vec::with_capacity(report.len() + 9);
        command.extend_from_slice(&self.command_register.to_le_bytes());
        if id < 0x0f {
            command.extend_from_slice(&[REPORT_TYPE_FEATURE << 4 | id, OPCODE_SET_REPORT]);
        } else {
            command.extend_from_slice(&[REPORT_TYPE_FEATURE << 4 | 0x0f, OPCODE_SET_REPORT, id]);
        }
        command.extend_from_slice(&self.data_register.to_le_bytes());
        command.extend_from_slice(&((report.len() + 2) as u16).to_le_bytes());
        command.extend_from_slice(report);
        self.bus.write(self.address, &command).map_err(Error::Bus)
    }

    fn push_motion(&mut self, motion: PointerReport) -> Result<(), Error<I::Error>> {
        if let Some(pending) = self.reports.back_mut()
            && pending.buttons == motion.buttons
        {
            pending.dx = pending.dx.saturating_add(motion.dx);
            pending.dy = pending.dy.saturating_add(motion.dy);
            pending.dz = pending.dz.saturating_add(motion.dz);
            Ok(())
        } else {
            self.reports
                .push_back(motion)
                .map_err(|_| Error::InvalidReport)
        }
    }

    /// Read the pending input reports into the pointer motion and key buffer.
    fn poll(&mut self) {
        let mut input = [0; MAX_INPUT];
        let input = &mut input[..self.max_input];
        for _ in 0..REPORTS_PER_POLL {
            // Bounded backpressure rather than dropping button transitions.
            if !self.enabled || self.reports.is_full() || !self.interrupt.is_pending() {
                break;
            }
            if let Err(error) = self.read_report(input) {
                // A broken device must not stall every input poll.
                log::warn!(
                    "HID I2C device at {:#x} disabled: {:?}",
                    self.address,
                    error
                );
                self.enabled = false;
                self.reports.clear();
                let _ = self.reports.push_back(PointerReport::default());
                self.keys.process_report(&KeyboardReport::default());
            }
        }
        self.keys.handle_repeat();
    }

    fn read_report(&mut self, input: &mut [u8]) -> Result<(), Error<I::Error>> {
        self.bus.read(self.address, input).map_err(Error::Bus)?;
        let length = usize::from(u16::from_le_bytes([input[0], input[1]]));
        // A zero length announces a reset and carries no report.
        if length == 0 {
            return Ok(());
        }
        if !(3..=input.len()).contains(&length) {
            return Err(Error::InvalidReport);
        }
        let report = &input[2..length];
        if let Some(mouse) = &self.mouse
            && matches(report, mouse.report_id, mouse.size)
            && let Some(motion) = mouse.read(report)
        {
            self.push_motion(motion)?;
        }
        if let Some(touchpad) = &self.touchpad
            && matches(report, touchpad.report_id, touchpad.size)
            && let Some((contact, count, button)) = touchpad.read(report)
        {
            let (dx, dy) = self
                .previous
                .zip(contact)
                .filter(|(old, new)| old.id == new.id)
                .map_or((0, 0), |(old, new)| (new.x - old.x, new.y - old.y));
            self.previous = contact;
            // Clickpad: a press with two fingers down is the right button.
            let buttons = match (button, count >= 2) {
                (false, _) => 0,
                (true, false) => 1,
                (true, true) => 2,
            };
            self.push_motion(PointerReport {
                dx,
                dy,
                dz: 0,
                buttons,
            })?;
        }
        if let Some(keyboard) = &self.keyboard
            && matches(report, keyboard.report_id, keyboard.size)
            && let Some(keys) = keyboard.read(report)
        {
            self.keys.process_report(&keys);
        }
        Ok(())
    }

    fn stop(&mut self) {
        if core::mem::replace(&mut self.enabled, false)
            && let Err(error) = self.command(OPCODE_SET_POWER, POWER_SLEEP)
        {
            log::warn!(
                "HID I2C device at {:#x}: sleep failed: {:?}",
                self.address,
                error
            );
        }
    }
}

/// The pointer function of a shared HID device.
struct HidPointer<I>(Rc<RefCell<HidI2c<I>>>);

impl<I: I2c> PointerDevice for HidPointer<I> {
    fn poll(&mut self) -> Option<PointerReport> {
        let mut device = self.0.borrow_mut();
        device.poll();
        let report = device.reports.pop_front().unwrap_or(PointerReport {
            buttons: device.buttons,
            ..PointerReport::default()
        });
        device.buttons = report.buttons;
        Some(report)
    }

    fn stop(&mut self) {
        self.0.borrow_mut().stop();
    }
}

/// The keyboard function of a shared HID device.
struct HidKeyboard<I>(Rc<RefCell<HidI2c<I>>>);

impl<I: I2c> KeyboardDevice for HidKeyboard<I> {
    fn read_key(&mut self) -> Option<Key> {
        let mut device = self.0.borrow_mut();
        device.poll();
        device.keys.read_key()
    }

    fn key_state(&mut self) -> KeyState {
        self.0.borrow().keys.efi_state()
    }

    fn stop(&mut self) {
        self.0.borrow_mut().stop();
    }
}

/// Whether `register` holds a HID descriptor right now (one read).
fn descriptor_ready<I: I2c>(bus: &mut I, address: u8, register: u16) -> bool {
    let mut raw = [0; size_of::<Descriptor>()];
    bus.write_read(address, &register.to_le_bytes(), &mut raw)
        .is_ok()
        && raw[..4] == [30, 0, 0, 1]
}

/// Register a ready device's pointer and keyboard functions.
fn register_opened(hid: HidI2c<I2cBus>) -> Option<HidPointer<I2cBus>> {
    let (mouse, keyboard) = (
        hid.mouse.is_some() || hid.touchpad.is_some(),
        hid.keyboard.is_some(),
    );
    let hid = Rc::new(RefCell::new(hid));
    if keyboard {
        super::keyboard_common::register(Box::new(HidKeyboard(hid.clone())));
    }
    mouse.then(|| HidPointer(hid))
}

/// A device that did not serve its HID descriptor at bind time. Some
/// touchpads (FocalTech on Google Navi) answer with zeros for seconds after
/// firmware takes over; keep asking from the pointer poll.
struct PendingHid {
    state: Pending,
    address: u8,
    register: u16,
    next: Timeout,
    tries: u32,
}

enum Pending {
    Waiting(I2cBus, Interrupt),
    Open(HidPointer<I2cBus>),
    Gone,
}

/// Retry interval while waiting for the descriptor.
const PENDING_RETRY_MS: u64 = 250;

impl PointerDevice for PendingHid {
    fn poll(&mut self) -> Option<PointerReport> {
        if let Pending::Open(pointer) = &mut self.state {
            return pointer.poll();
        }
        if !matches!(self.state, Pending::Waiting(..)) || !self.next.is_expired() {
            return None;
        }
        self.tries += 1;
        self.next = Timeout::from_ms(PENDING_RETRY_MS);
        let Pending::Waiting(bus, _) = &mut self.state else {
            return None;
        };
        if !descriptor_ready(bus, self.address, self.register) {
            if self.tries.is_multiple_of(40) {
                log::debug!(
                    "HID I2C {:#x}: still no descriptor after {} retries",
                    self.address,
                    self.tries
                );
            }
            return None;
        }
        log::debug!(
            "HID I2C {:#x}: descriptor appeared after {} retries ({} ms)",
            self.address,
            self.tries,
            u64::from(self.tries) * PENDING_RETRY_MS
        );
        let Pending::Waiting(bus, interrupt) = core::mem::replace(&mut self.state, Pending::Gone)
        else {
            return None;
        };
        match HidI2c::open(bus, self.address, interrupt, self.register) {
            Ok(hid) => {
                if let Some(pointer) = register_opened(hid) {
                    self.state = Pending::Open(pointer);
                }
            }
            Err(error) => log::warn!("HID I2C device at {:#x}: {:?}", self.address, error),
        }
        None
    }

    fn stop(&mut self) {
        if let Pending::Open(pointer) = &mut self.state {
            pointer.stop();
        }
    }
}

/// Bind a described HID-over-I2C device as pointer and/or keyboard.
pub(crate) fn bind(device: I2cPeripheral) {
    let I2cPeripheral {
        mut bus,
        address,
        interrupt,
        hid_descriptor_register,
        ..
    } = device;
    let (interrupt, register) = match (interrupt, hid_descriptor_register) {
        (None, _) => {
            return log::warn!(
                "HID I2C device at {address:#x}: {:?}",
                Error::<()>::NoInterrupt
            );
        }
        (_, None) => {
            return log::warn!(
                "HID I2C device at {address:#x}: {:?}",
                Error::<()>::NoDescriptorRegister
            );
        }
        (Some(interrupt), Some(register)) => (interrupt, register),
    };
    if !descriptor_ready(&mut bus, address, register) {
        log::info!("HID I2C {address:#x}: no descriptor yet; retrying in the background");
        super::pointer::register(Box::new(PendingHid {
            state: Pending::Waiting(bus, interrupt),
            address,
            register,
            next: Timeout::from_ms(PENDING_RETRY_MS),
            tries: 0,
        }));
        return;
    }
    match HidI2c::<I2cBus>::open(bus, address, interrupt, register) {
        Ok(hid) => {
            if let Some(pointer) = register_opened(hid) {
                super::pointer::register(Box::new(pointer));
            }
        }
        Err(error) => log::warn!("HID I2C device at {address:#x}: {:?}", error),
    }
}

#[cfg(test)]
#[path = "hid_i2c_tests.rs"]
mod tests;
