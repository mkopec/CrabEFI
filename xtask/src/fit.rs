//! Package the AArch64 payload as a FIT for coreboot (`PAYLOAD_FIT`).
//!
//! coreboot's FIT loader treats the kernel image as a Linux arm64 Image,
//! loads it at the FIT `load` address (the payload link address) and passes
//! the devicetree whose root `compatible` best matches the board and SKU.

use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result, bail};

/// Build `<elf>.fit` with the given devicetrees; returns its path.
pub fn package(elf: &Path, load_address: u64, dtbs: &[PathBuf]) -> Result<PathBuf> {
    if dtbs.is_empty() {
        bail!("a FIT needs at least one devicetree");
    }
    let work = tempfile::tempdir()?;
    let image = work.path().join("crabefi.bin");
    run(Command::new("llvm-objcopy")
        .args(["-O", "binary"])
        .arg(elf)
        .arg(&image))?;
    let raw = fs::read(&image)?;
    if raw.get(0x38..0x3c) != Some(b"ARM\x64") {
        bail!("{} has no arm64 Image header", elf.display());
    }

    // coreboot's ulzman() reads the legacy .lzma header and rejects the
    // "unknown size" marker xz writes, so store the real size.
    let compressed = work.path().join("crabefi.lzma");
    run(Command::new("xz")
        .args(["--format=lzma", "--keep", "--stdout", "-9"])
        .arg(&image)
        .stdout(fs::File::create(&compressed)?))?;
    let mut lzma = fs::read(&compressed)?;
    lzma[5..13].copy_from_slice(&(raw.len() as u64).to_le_bytes());
    fs::write(&compressed, lzma)?;

    let mut images = String::new();
    let mut configurations = String::new();
    for (index, dtb) in dtbs.iter().enumerate() {
        let dtb = dtb
            .canonicalize()
            .with_context(|| format!("devicetree {}", dtb.display()))?;
        let name = dtb.file_name().unwrap_or_default().to_string_lossy();
        let n = index + 1;
        images += &format!(
            "\n        fdt-{n} {{\n            description = \"{name}\";\n            \
             data = /incbin/(\"{}\");\n            type = \"flat_dt\";\n            \
             arch = \"arm64\";\n            compression = \"none\";\n        }};",
            dtb.display()
        );
        configurations += &format!(
            "\n        conf-{n} {{\n            kernel = \"kernel\";\n            \
             fdt = \"fdt-{n}\";\n        }};"
        );
    }
    let its = work.path().join("crabefi.its");
    fs::write(
        &its,
        format!(
            "/dts-v1/;\n/ {{\n    description = \"CrabEFI\";\n    #address-cells = <1>;\n    \
             images {{\n        kernel {{\n            description = \"CrabEFI\";\n            \
             data = /incbin/(\"{}\");\n            type = \"kernel\";\n            \
             arch = \"arm64\";\n            os = \"linux\";\n            \
             compression = \"lzma\";\n            load = <{:#x}>;\n            \
             entry = <{:#x}>;\n        }};{images}\n    }};\n    configurations {{\n        \
             default = \"conf-1\";{configurations}\n    }};\n}};\n",
            compressed.display(),
            load_address,
            load_address
        ),
    )?;
    let output = elf.with_extension("fit");
    run(Command::new("mkimage")
        .arg("-q")
        .arg("-f")
        .arg(&its)
        .arg(&output))
    .context("mkimage (u-boot-tools) is required for --fit-dtb")?;
    Ok(output)
}

fn run(command: &mut Command) -> Result<()> {
    let program = command.get_program().to_string_lossy().into_owned();
    let status = command
        .status()
        .with_context(|| format!("failed to run {program}"))?;
    if !status.success() {
        bail!("{program} failed");
    }
    Ok(())
}
