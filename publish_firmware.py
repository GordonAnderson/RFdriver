"""
PlatformIO post-build step: copies the built firmware.bin into firmware/,
named RFdriver_v<version>.bin, where <version> is parsed straight out of the
Version[] string in src/RFdriver.cpp - so the file that gets published always
matches what the firmware itself reports via GVER, with no separate manual
step to forget. Also keeps the table in firmware/README.md up to date with
the size, CRC-8 (matching ComputeCRC() in Hardware.cpp / the MIPS host app's
CalculateCRC()), and SHA-256 of that exact file - the values the PGM command
protocol and a recovery flash both need.

See firmware/README.md for what this produces and how to use it; see
platformio.ini (extra_scripts) for how this gets invoked.
"""
Import("env")

import hashlib
import os
import re

PROJECT_DIR = env["PROJECT_DIR"]
SRC_FILE = os.path.join(PROJECT_DIR, "src", "RFdriver.cpp")
FIRMWARE_DIR = os.path.join(PROJECT_DIR, "firmware")
README_FILE = os.path.join(FIRMWARE_DIR, "README.md")

# Application partition size - must track APP_FLASH_START/APP_FLASH_END in
# include/Hardware.h. A build larger than this cannot be installed by PGM.
APP_FLASH_START = 0x00002000
APP_FLASH_END = 0x00021000
APP_MAX_SIZE = APP_FLASH_END - APP_FLASH_START

VERSION_RE = re.compile(r'Version\[\]\s*PROGMEM\s*=\s*"RFdriver version ([^"]+)"')
TABLE_ROW_RE = r'\| `{filename}` \|.*\|\n'
TABLE_HEADER_RE = re.compile(r'(\|---\|---\|---\|---\|---\|\n)')


def compute_crc8(data, generator=0x1D):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ generator) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def get_version_label():
    with open(SRC_FILE, "r") as f:
        content = f.read()
    m = VERSION_RE.search(content)
    if not m:
        return None, None
    label = m.group(1).strip()          # e.g. "1.4, September 7, 2026"
    short = label.split(",")[0].strip() # e.g. "1.4"
    return short, label


def update_readme_table(filename, version_label, size, crc8, sha256):
    if not os.path.isfile(README_FILE):
        return  # Nothing to update if the file's been removed; don't recreate it here.
    with open(README_FILE, "r") as f:
        text = f.read()
    row = "| `{}` | {} | {} | {} | `{}` |\n".format(
        filename, version_label, size, crc8, sha256
    )
    existing_row_re = re.compile(TABLE_ROW_RE.format(filename=re.escape(filename)))
    if existing_row_re.search(text):
        text = existing_row_re.sub(row, text)
    elif TABLE_HEADER_RE.search(text):
        text = TABLE_HEADER_RE.sub(r"\1" + row, text, count=1)
    else:
        return  # Table structure not found/changed - leave README alone rather than guess.
    with open(README_FILE, "w") as f:
        f.write(text)


def publish_firmware(source, target, env):
    # AddPostAction's "source" is what produced the target (firmware.elf, via
    # objcopy) - the actual built firmware.bin we want is "target".
    short_version, version_label = get_version_label()
    if not short_version:
        print("firmware publish: could not find Version[] in src/RFdriver.cpp - skipped")
        return

    built_bin = str(target[0])
    with open(built_bin, "rb") as f:
        data = f.read()

    # The PGM field update copies a staged image over the application
    # partition, so the build has to fit in that partition - not merely in the
    # chip. These must track APP_FLASH_START/APP_FLASH_END in include/Hardware.h.
    if len(data) > APP_MAX_SIZE:
        raise Exception(
            "firmware publish: build is {} bytes, which does not fit the {}-byte "
            "application partition (APP_FLASH_START..APP_FLASH_END in "
            "include/Hardware.h). PGM could not install this image. Either shrink "
            "the build or re-partition - note the staging region must stay at "
            "least as large as the application region.".format(len(data), APP_MAX_SIZE)
        )

    os.makedirs(FIRMWARE_DIR, exist_ok=True)
    filename = "RFdriver_v{}.bin".format(short_version)
    dest = os.path.join(FIRMWARE_DIR, filename)

    if os.path.isfile(dest):
        with open(dest, "rb") as f:
            old_data = f.read()
        if old_data != data:
            print(
                "firmware publish: WARNING - overwriting {} with DIFFERENT content "
                "for the same version label. Did you forget to bump Version[] in "
                "src/RFdriver.cpp?".format(filename)
            )

    with open(dest, "wb") as f:
        f.write(data)

    size = len(data)
    crc8 = compute_crc8(data)
    sha256 = hashlib.sha256(data).hexdigest()
    update_readme_table(filename, version_label, size, crc8, sha256)

    print(
        "firmware publish: {} (size={}, crc8={}, sha256={})".format(
            dest, size, crc8, sha256
        )
    )


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", publish_firmware)
