# RFdriver Hardware Test Checklist

Post-PlatformIO-port validation. Covers the one framework change that
couldn't be verified without hardware (`Wire.onAddressMatch()`, see
[lib/Wire/](lib/Wire/) and [README.md](README.md)) and the bugs fixed during
review (see the Rev 1.4 entry in [src/RFdriver.cpp](src/RFdriver.cpp)).

Fill in Pass/Fail and notes as you go. If anything misbehaves, start with
item 2 (TWI address-match) - it's the one piece of framework code that was
patched rather than just read.

## 1. USB serial sanity

- [ ] Connect over USB, open a serial terminal at 115200 baud
- [ ] Sign-on banner prints on reset (`RFdriver version 1.3, ...`)
- [ ] `GVER` reports version
- [ ] `GCMDS` lists the full command set

Notes:

## 2. TWI address-match (highest priority - see README)

From the MIPS controller (or a bench I2C master):

- [ ] Read/write SEPROM page 1 (base address)
- [ ] Read/write SEPROM page 2 (base address + 1)
- [ ] Command address works (base | 0x20, or | 0x18 for extended addressing)
- [ ] Repeat with jumpers set for extended addressing, if applicable

Notes:

## 3. RF frequency (`SRFFRQ` / `GRFFRQ`, TWI_SET_FREQ / TWI_READ_FREQ)

- [ ] Channel 1: set 400kHz, 1MHz, 5MHz - confirm on scope/counter
- [ ] Channel 2: set 400kHz, 1MHz, 5MHz - confirm on scope/counter
- [ ] Frequency readback (`GRFFRQ`) matches what was set

Notes:

## 4. RF drive level (`SRFDRV` / `GRFDRV`, TWI_SET_DRIVE / TWI_READ_DRIVE)

- [ ] Channel 1: 0%, 50%, 100% - confirm PWM duty cycle and resulting drive
- [ ] Channel 2: 0%, 50%, 100% - confirm PWM duty cycle and resulting drive

Notes:

## 5. Readbacks (`GRFPPVP` / `GRFPPVN` / `GRFDRV` / `GRFPWR`)

- [ ] RF+ level readback matches a known reference signal
- [ ] RF- level readback matches a known reference signal
- [ ] Power readback is sane at a known drive level

Notes:

## 6. Bug-fix regression checks

- [ ] `RFCALP <ch>,0` and `RFCALN <ch>,0` reset gain to default (32) instead
      of corrupting `m` (previously: divide-by-zero on `vpp == 0`)
- [ ] `RFPWLC`: enter the same measured voltage/drive level for two
      consecutive points (or otherwise force two points to read the same raw
      ADC count) - should print a rejection message and re-prompt for the
      same point, not silently accept a duplicate
- [ ] TWI `TWI_READ_PWL` (0x88) read does not leak into `TWI_READ_PWL_N`
      (0x89) data on the next transaction
- [ ] `RRFCH2` reports "RF channel 2 values" (not "channel 1")

Notes:

## 7. Gate input (`TWI_SET_GATENA` / `TWI_SET_GATE` / `TWI_SET_GATEDIS`)

- [ ] Channel 1: configured DIO pin gates drive off/on as expected
- [ ] Channel 2: configured DIO pin gates drive off/on as expected
- [ ] `TWI_SET_GATEDIS` disables the gate interrupt cleanly

Notes:

## 8. Auto tune (`TUNERFCH` / `RETUNERFCH`)

- [ ] `TUNERFCH` on channel 1 completes and lands on a sensible frequency
- [ ] `RETUNERFCH` on channel 1 (from a known-good frequency) converges
- [ ] Repeat both on channel 2

Notes:

## 9. Persistence (`SAVE` / `RESTORE` / `FORMAT`)

- [ ] `SAVE`, power-cycle, settings are restored on boot
- [ ] `RESTORE` reloads the last saved settings
- [ ] `FORMAT`, confirm it resets to Rev 1 defaults

Notes:

## 10. Firmware field update over direct USB (`PGM` command) - test LAST, on a recoverable bench unit

New feature, never run on real hardware before. Do this only after every
other section above passes, on a unit you're set up to recover if something
goes wrong - not a unit that's already installed.

This is direct-USB only - proving the on-module update mechanism itself
works before it's ever relied on. Doing the same thing through the MIPS TWI
relay (so a case never has to be opened at all) is separate, not-yet-built
work - see [todo.md](todo.md).

**Before starting**, confirm the recovery path works on this exact board:
double-tap its reset button, confirm a new serial port appears (its SAM-BA
bootloader), and confirm `pio run -t upload` (or `bossac` directly) can
reflash it from that state. Use [firmware/RFdriver_v1.4.bin](firmware/RFdriver_v1.4.bin)
(size 77952, CRC-8 112 - see [firmware/README.md](firmware/README.md)) as
the known-good image, both for this recovery check and for the successful
transfer test below.

- [ ] Recovery path (above) confirmed working *before* attempting an update
- [ ] `PGM,<size>` with a deliberately-bad file (e.g. truncate one byte, or
      flip a byte in the CRC line): update is rejected, board keeps running
      the old firmware with no reset needed
- [ ] `PGM,<size>` with a file that isn't a valid image at all (e.g. a text
      file): rejected immediately as "does not look like a valid firmware
      image", nothing written
- [ ] `PGM,77952` with `firmware/RFdriver_v1.4.bin` and CRC `112`: transfer
      completes, board reports success and resets, `GVER` afterward reports
      "RFdriver version 1.4, September 7, 2026" - i.e. a real update
      round-trip works
- [ ] RF drive on both channels reads 0 immediately once the transfer starts
      (confirms the safety-off happens before anything else)
- [ ] Time the transfer for a realistic-sized image, note it below
- [ ] If anything goes wrong: confirm the recovery path (above) still works

Notes:

## Overall result

- [ ] All sections pass
- [ ] Issues found (list below, with section/item numbers)
