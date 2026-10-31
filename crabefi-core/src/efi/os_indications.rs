//! `OsIndications` requests that are not about capsules.
//!
//! Capsule delivery bits stay with the capsule code (`capsule::disk`).
//!
//! Reference: UEFI Specification 2.10, Section 8.5.4.

use crabefi_efi_types::secure_boot::EFI_GLOBAL_VARIABLE_GUID;
use r_efi::efi::Guid;

/// `OsIndications` bit asking the firmware to stop at its setup UI.
pub const EFI_OS_INDICATIONS_BOOT_TO_FW_UI: u64 = 0x0000_0000_0000_0001;

/// `OsIndications`, NUL-terminated UTF-16.
const OS_INDICATIONS: &[u16] = &[
    'O' as u16, 's' as u16, 'I' as u16, 'n' as u16, 'd' as u16, 'i' as u16, 'c' as u16, 'a' as u16,
    't' as u16, 'i' as u16, 'o' as u16, 'n' as u16, 's' as u16, 0,
];

/// The value to store back if `value` requests the firmware UI.
fn without_boot_to_fw_ui(value: u64) -> Option<u64> {
    (value & EFI_OS_INDICATIONS_BOOT_TO_FW_UI != 0)
        .then_some(value & !EFI_OS_INDICATIONS_BOOT_TO_FW_UI)
}

/// Consume a `BOOT_TO_FW_UI` request from `OsIndications`.
///
/// Returns whether the OS asked to stop at the firmware UI on this boot
/// (`systemctl reboot --firmware-setup`). The bit is cleared first so the
/// request is one-shot.
pub fn take_boot_to_fw_ui_request() -> bool {
    let guid = Guid::from_bytes(&EFI_GLOBAL_VARIABLE_GUID);
    let Some((attributes, data)) =
        crate::efi::runtime_image::client::variables::get(&guid, OS_INDICATIONS)
    else {
        return false;
    };
    let Some(value) = data
        .get(..8)
        .and_then(|bytes| bytes.try_into().ok())
        .map(u64::from_le_bytes)
        .and_then(without_boot_to_fw_ui)
    else {
        return false;
    };
    let status = crate::efi::runtime_image::client::variables::set(
        &guid,
        OS_INDICATIONS,
        attributes,
        &value.to_le_bytes(),
    );
    if status != r_efi::efi::Status::SUCCESS {
        log::warn!("Failed to clear BOOT_TO_FW_UI in OsIndications: {status:?}");
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boot_to_fw_ui_is_cleared_and_other_bits_kept() {
        assert_eq!(without_boot_to_fw_ui(0), None);
        assert_eq!(without_boot_to_fw_ui(0x4), None);
        assert_eq!(without_boot_to_fw_ui(0x1), Some(0));
        assert_eq!(without_boot_to_fw_ui(0x5), Some(0x4));
    }
}
