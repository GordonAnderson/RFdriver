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

`PGM` (`ProgramFLASHcmd()` in [src/Hardware.cpp](src/Hardware.cpp)) lets a
USB-connected host replace this module's own firmware without a debugger,
reusing the same hex-encoded, CRC-checked transfer protocol the MIPS host
tooling already speaks to other modules (`Comms::ARBupload()` in the MIPS
host app): send `PGM,<size>` (image size in bytes, decimal), then the image
as ASCII hex (two characters per byte), then a newline and the image's 8-bit
CRC (poly 0x1D) in decimal.

This board has one flash bank, so the update writes over the running
application in place - see the detailed safety design and its limits in the
comment above `ProgramFLASHcmd()`. The short version:

- The image is validated (size bounds, a plausible vector table) before
  anything is written, and every row is read back and verified against what
  was sent.
- The vector table (the image's first 256 bytes) is deliberately held back
  in RAM and committed - by a small, separately-verified, RAM-resident
  routine - only after the *entire rest* of the image has been written and
  its whole-image CRC has checked out. That commit is immediately followed
  by a reset into the new firmware and never returns.
- Consequently, any failure detected along the way (bad CRC, a verify
  mismatch, a timeout, an implausible image) leaves the *old* firmware's
  vector table untouched and the board keeps running normally - no recovery
  needed for the common failure modes.
- What it does not protect against: a failure (power loss, a dropped USB
  connection) in the middle of writing one of the other rows can still leave
  the running application internally inconsistent. Recovery from that is the
  same physical bootloader recovery this board already needs for its initial
  programming: open the enclosure, double-tap reset to force it into its
  SAM-BA bootloader (it reappears as a serial port - this board's bootloader
  is the classic Arduino/`bossac` one, not the UF2/drag-and-drop kind some
  other Adafruit SAMD boards use), then reflash a known-good build with
  `pio run -t upload` or `bossac` directly. Nothing `PGM` does can affect
  that recovery path, since it never touches flash below the application's
  own start address.

This has been reviewed carefully and checked at the disassembly level, but
has not yet been exercised on real hardware - see [checklist.md](checklist.md).

Every `pio run` automatically publishes the build to
[firmware/](firmware/) as `RFdriver_v<version>.bin`, ready to hand to `PGM`
or use as a recovery image - see [firmware/README.md](firmware/README.md).
