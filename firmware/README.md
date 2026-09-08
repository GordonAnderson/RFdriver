# RFdriver firmware images

Built binaries for the `PGM` field-update command (see the "Firmware field
update" section in [../README.md](../README.md) and
[../src/Hardware.cpp](../src/Hardware.cpp)'s `ProgramFLASHcmd()`). Also usable
as a known-good image for the direct USB/`bossac` recovery path.

Naming: `RFdriver_v<version>.bin`, where `<version>` matches the version
`GVER` reports at runtime (the `Version[]` string in
[../src/RFdriver.cpp](../src/RFdriver.cpp)) - keep those two in sync when
cutting a new build; a firmware image that can't tell you its own version
number defeats the point of naming the file after one.

## Current build

| File | Version | Size (bytes) | CRC-8 (poly 0x1D) | SHA-256 |
|---|---|---|---|---|
| `RFdriver_v1.5.bin` | 1.5, September 8, 2026 | 77984 | 106 | `fb0f7bfff9e25c14dcce35f1d1eaf47279574d2548fbaf7174b225506717d49c` |
| `RFdriver_v1.4.bin` | 1.4, September 7, 2026 | 77952 | 112 | `fe876840f1d0cee621ddb863e7318fd2381c0a05c12be45d6fb9b007d5b16a96` |

Size and CRC-8 are exactly the two values the `PGM` command's transfer
protocol needs (`PGM,<size>`, then the hex-encoded image, then a newline and
the CRC in decimal) - useful to have on hand rather than recomputing them
mid-test. The CRC uses the same algorithm as `ComputeCRC()` in
[../src/Hardware.cpp](../src/Hardware.cpp) and `Comms::CalculateCRC()` in the
MIPS host app, so a value computed either way should always agree.

## Cutting a new version

This folder and the table above are kept up to date automatically by
[../publish_firmware.py](../publish_firmware.py) (wired in via
`extra_scripts` in [../platformio.ini](../platformio.ini)): every `pio run`
copies the freshly-built binary in here as `RFdriver_v<version>.bin` and
rewrites this table's row for it, computing the size, CRC-8, and SHA-256
itself. There's nothing to do by hand except:

1. Bump the `Version[]` string in [../src/RFdriver.cpp](../src/RFdriver.cpp)
   *before* building - the script reads the version from there, so a build
   with a stale version string publishes under the old file name and
   silently overwrites it (it does print a warning to the build log if the
   content actually changed under an unchanged version label, but bumping
   first avoids the question entirely).
2. `pio run`.

Old versions aren't deleted automatically - remove a superseded
`RFdriver_v<old>.bin` by hand if you don't want it kept around, and delete
its row from the table above.
