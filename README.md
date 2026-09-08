# RFdriver

Firmware for the MIPS **RFdriver** module (rev 6.0 hardware), built on an
Adafruit Feather M0 (SAMD21G18A). This module drives two independent RF
channels - each with its own frequency source, drive-level PWM, and RF+/RF-
level readback - and presents itself to a MIPS controller as an emulated
SEPROM over TWI (I2C), the same interface earlier RFdriver hardware
revisions used. It can also run standalone over USB serial, using the same
command syntax as MIPS.

See the comment block at the top of [src/RFdriver.cpp](src/RFdriver.cpp) for
the full revision history and protocol background - that's the canonical
changelog for this firmware and is kept up to date there rather than here.

## Hardware

- MCU: SAMD21G18A (Adafruit Feather M0 board definition)
- AD5592R: SPI-connected 8-channel ADC/DAC/DIO chip, used for all analog
  drive-level and readback channels (see `RFdriverAD5592init()` in
  [src/RFdriver.cpp](src/RFdriver.cpp) for the channel assignment)
- CY22393: TWI-connected PLL clock generator (bit-banged software I2C, since
  the hardware TWI is used for the MIPS interface), driving each RF channel's
  frequency ([src/ClockGenerator.cpp](src/ClockGenerator.cpp))
- Two RF channel drive outputs, PWM-controlled via the SAMD21's TCC1/TCC2
  timers ([src/Hardware.cpp](src/Hardware.cpp))

## Repository layout

```
src/                 Firmware source
  RFdriver.cpp        Main sketch: setup()/loop(), TWI protocol handling,
                       the RF control loop, and host (USB) command handlers
  Hardware.cpp        AD5592 driver, drive PWM setup, FLASH firmware update
  ClockGenerator.cpp  CY22393 PLL driver
  Calibration.cpp     Interactive AD5592 channel calibration helpers
                       (not currently wired to a host command)
  Serial.cpp          Host command line: ring buffer, tokenizer, command
                       table (CmdArray) and dispatcher
  Hooks.c             SysTick hook glue used by msTimerIntercept()
include/              Headers for the above, plus shared MIPS ecosystem
                       definitions (Errors.h) and a couple of vendored
                       utility headers (AtomicBlock.h)
lib/Wire/             Project-local override of the framework's Wire
                       library - see "Notes on this PlatformIO port" below
platformio.ini        Build configuration
```

## Building

This project uses [PlatformIO](https://platformio.org/); it does not need
the Arduino IDE.

```sh
pio run              # build
pio run -t upload    # build and flash over USB
pio device monitor    # open a serial terminal (115200 baud)
```

Or open the folder in VS Code with the PlatformIO extension installed.

## Notes on this PlatformIO port

This project was originally developed in the Arduino IDE and was ported to
PlatformIO. Two things in [platformio.ini](platformio.ini) and
[lib/Wire/](lib/Wire/) exist specifically because of that port and are worth
understanding before changing them:

- **`lib/Wire/`** is a project-local copy of the framework's `Wire` library
  with `TwoWire::onAddressMatch()` added back in. The SAMD core version
  PlatformIO installs for this board is missing that method, even though the
  Arduino IDE's installed core has it; `setup()` needs it to tell which of
  this module's several TWI addresses (SEPROM page 1, page 2, or the command
  address) was actually addressed. The added code is a small, direct port of
  the same method from the Arduino IDE's core, not a rewrite.
- **`lib_ignore = Adafruit TinyUSB Library`** in platformio.ini works around
  an unrelated packaging quirk: this board never uses TinyUSB (that library
  only builds when a TinyUSB USB stack is selected), but PlatformIO's
  dependency scanner pulls its sources in anyway and they fail to compile.
  Excluding it is the standard fix for Feather M0 projects on this core.

Neither of these changes any runtime behavior versus the original Arduino
build; they exist purely to make that same firmware build under PlatformIO.

## Firmware architecture

- `setup()` loads the saved configuration from flash (falling back to
  defaults if none is present/valid), brings up the TWI slave interface,
  drive PWM, and clock generator, and starts a `Thread` (via the
  ArduinoThread library) that calls `Update()` every 25 ms.
- `Update()` is the real control loop: it drives each channel's PWM level
  and PLL frequency when they've changed, refreshes the ADC readbacks, runs
  the closed-loop RF level control (`VRFcontrolLoop()`, for channels in
  `RF_AUTO` mode) and the auto-tune state machine (`RFdriver_tune()`).
- `loop()` services the USB command line and the ThreadController, and
  performs the deferred work that TWI command handlers can't safely do from
  inside the I2C interrupt context (gain calibration, PWL table capture).
- The module answers to up to three TWI addresses derived from its base
  address (jumper bits ORed in): the base address (SEPROM page 1), base+1
  (SEPROM page 2), and base|0x20 (or |0x18 for extended addressing) for the
  command interface. See the header comment in
  [src/RFdriver.cpp](src/RFdriver.cpp) and the `TWI_*` command definitions in
  [include/RFdriver.h](include/RFdriver.h) for the full protocol.
- The USB host command set (see `CmdArray` in
  [src/Serial.cpp](src/Serial.cpp)) uses the same syntax as MIPS; `GCMDS`
  lists all commands from a connected terminal.

## Firmware field update (`PGM` command)

`PGM` (`ProgramFLASHcmd()` in [src/Hardware.cpp](src/Hardware.cpp)) replaces
this module's own firmware without a debugger - either from a host on the
module's own USB port, or from a MIPS controller relaying over TWI
(`TWITALK`), which needs no physical access to the module at all. It reuses
the same hex-encoded, CRC-checked transfer protocol the MIPS host tooling
already speaks to other modules (`Comms::ARBupload()` in the MIPS host app):
send `PGM,<size>` (image size in bytes, decimal), then the image as ASCII hex
(two characters per byte), then a newline and the image's 8-bit CRC (poly
0x1D) in decimal. Drive it with [pgm_update.py](pgm_update.py).

> **Working as of Rev 1.5** (September 8, 2026), verified on hardware both
> over direct USB and through the MIPS TWI relay - which is the point of the
> feature: a module's firmware can be updated without opening the enclosure.
> Direct USB: full 77984-byte round trip in ~13 s, a real v1.5 -> v1.4 version
> change, and clean rejection of bad-CRC / non-image / dropped-transfer cases
> with the board still running. Through `TWITALK`: same image in 162 s, plus a
> rejected transfer that left the module running. See
> [checklist.md](checklist.md) §10 and §11.
>
> **Rev 1.4's `PGM` was broken** - it hard-hung the board on every update and
> needed physical bootloader recovery. Do not field 1.4. The postmortem below
> explains why, because the failure mode is easy to reintroduce.

### Why the Rev 1.4 design failed

This board has one flash bank, so the update as originally written overwrote
the running application in place. That is the fatal flaw: **it erased the
code it was currently executing.**

`ProgramFLASHcmd()` links at `0x292C-0x2CDB` (flash rows 9-12), and its
erase/write sequence sits at `0x2ad0-0x2ae8`, inside row 10. When the update
reaches row 10:

1. `noInterrupts()` disables interrupts.
2. `FlashClass::erase()` wipes row 10 to `0xFF`.
3. `erase()` returns to `0x2ae0` - *an address inside the row it just
   erased*.
4. The CPU fetches `0xFFFF`, an undefined Thumb encoding, and HardFaults with
   interrupts off - before ever reaching the `write()` at `0x2ae8` that would
   have restored those bytes.

The board hangs until a physical reset. This is deterministic and reproduces
on every attempt, independent of image content, transfer length, or transfer
speed.

Rows 0-9 survive only by luck of code layout: while they are being erased,
the erase/write window itself (row 10) is still intact, and each row is
restored to identical content before anything in it is needed again. Row 10
is simply the first row that contains the critical window. At least 15
distinct rows hold code the update loop executes - `ProcessSerial`, `RB_Get`,
`GetToken`, `FlashClass`, `Print::println`, `millis`, `memcpy`, `siscanf`,
plus the whole USB/CDC stack - so dodging row 10 alone would not help.

The design comment above `ProgramFLASHcmd()` states the governing rule
correctly - *nothing in flash can be safely called into once we've started
rewriting it* - but applies it only to the row-0 commit, when it applies to
every row.

What did work, and is worth keeping: the RAM-resident
`IAP_CommitRow0AndReset()` was exercised successfully on real hardware (a
10-row transfer that stops short of row 10 completes, commits, resets, and
reboots cleanly). The RAM-resident commit technique is validated; it is the
in-place *streaming* write that is unworkable.

### The Rev 1.5 design - stage in spare flash, then RAM-resident copy

Writing to flash that holds no executing code is safe - that is exactly why
rows 0-9 worked. So flash is now partitioned (see
[include/Hardware.h](include/Hardware.h)):

| Region | Range | Size | Notes |
|---|---|---|---|
| Bootloader | `0x00000` - `0x02000` | 8KB | SAM-BA. Never touched; it is the recovery path. |
| Application | `0x02000` - `0x21000` | 124KB | The running firmware. Only ever written by the RAM-resident copier. |
| Staging | `0x21000` - `0x40000` | 124KB | Where an update is received and verified. Holds no code. |

1. The incoming image is streamed into **staging**. Serial, USB, CRC and the
   command loop all keep running from flash, because nothing being erased is
   ever executing. Each row is read back and verified as it lands.
2. The whole staged image is then re-read from flash and CRC'd again, which
   catches a row that verified but landed in the wrong place.
3. Only then does `IAP_CopyStagingToAppAndReset()` - RAM-resident, calling
   nothing in flash - copy staging over the application row by row and reset.

Any failure before step 3 (bad CRC, verify mismatch, timeout, implausible
image, dropped connection) leaves the application partition completely
untouched and the board running normally. That is the property Rev 1.4
claimed and did not have; it is now tested rather than argued.

The window in which a power loss is unrecoverable shrinks from the whole
multi-minute transfer to the few hundred milliseconds of flash-to-flash copy.

Two constraints this creates, both enforced or documented rather than left to
memory:

- **The build must fit the application partition**, not merely the chip.
  [publish_firmware.py](publish_firmware.py) fails the build if it doesn't.
  Staging must stay at least as large as the application partition.
- **`IAP_CopyStagingToAppAndReset()` must never call into flash.** After
  changing it, confirm with `arm-none-eabi-nm` that it still links at a
  `0x20xxxxxx` (RAM) address, and with `arm-none-eabi-objdump -D` that its
  disassembly contains no `bl` to a `0x0000xxxx` address. The `volatile`
  pointers in its copy loop are load-bearing: without them GCC emits a call
  to `memcpy`, which lives in flash, and the update would fault.

A `PGM` update resets saved settings, exactly as a `bossac` reflash does -
the `FlashStorage` backing store is linked into the image itself.

Recovery, in the meantime and after any failed update, is the physical
bootloader path this board already needs for initial programming: open the
enclosure, double-tap reset to force it into its SAM-BA bootloader (it
reappears as a serial port - this board's bootloader is the classic
Arduino/`bossac` one, not the UF2/drag-and-drop kind some other Adafruit SAMD
boards use), then reflash a known-good build. Nothing `PGM` does can affect
that recovery path, since it never touches flash below the application's own
start address.

`pio run -t upload` is *not* the tool to recover with: it does a 1200-baud
touch-reset that assumes the application is still running, so it fails
outright from an already-bootloader state (and was also seen to fail
mid-write once). Drive `bossac` directly instead - this exact invocation is
known to work from a bootloader state:

```
~/.platformio/packages/tool-bossac/bossac --port=cu.usbmodemXXXX \
    -e -w -v -R -o 0x2000 .pio/build/adafruit_feather_m0/firmware.bin
```

[pgm_update.py](pgm_update.py) is the host-side tool to drive `PGM` with (a
plain terminal can't type the hex-encoded image body); it also carries the
diagnostic flags used to characterise this failure.

Every `pio run` automatically publishes the build to
[firmware/](firmware/) as `RFdriver_v<version>.bin`, ready to hand to `PGM`
or use as a recovery image - see [firmware/README.md](firmware/README.md).
