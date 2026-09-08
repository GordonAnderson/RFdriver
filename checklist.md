# RFdriver Hardware Test Checklist

Post-PlatformIO-port validation. Covers the one framework change that
couldn't be verified without hardware (`Wire.onAddressMatch()`, see
[lib/Wire/](lib/Wire/) and [README.md](README.md)) and the bugs fixed during
review (see the Rev 1.4 entry in [src/RFdriver.cpp](src/RFdriver.cpp)).

Fill in Pass/Fail and notes as you go. If anything misbehaves, start with
item 2 (TWI address-match) - it's the one piece of framework code that was
patched rather than just read.

## 1. USB serial sanity

- [X] Connect over USB, open a serial terminal at 115200 baud
- [X] Sign-on banner prints on reset (`RFdriver version 1.3, ...`)
- [X] `GVER` reports version
- [X] `GCMDS` lists the full command set

Notes:

## 2. TWI address-match (highest priority - see README)

From the MIPS controller (or a bench I2C master):

- [X] Read/write SEPROM page 1 (base address)
- [X] Read/write SEPROM page 2 (base address + 1)
- [X] Command address works (base | 0x20, or | 0x18 for extended addressing)
- [ ] Repeat with jumpers set for extended addressing, if applicable

Notes:

## 3. RF frequency (`SRFFRQ` / `GRFFRQ`, TWI_SET_FREQ / TWI_READ_FREQ)

- [X] Channel 1: set 400kHz, 1MHz, 5MHz - confirm on scope/counter
- [X] Channel 2: set 400kHz, 1MHz, 5MHz - confirm on scope/counter
- [X] Frequency readback (`GRFFRQ`) matches what was set

Notes:

## 4. RF drive level (`SRFDRV` / `GRFDRV`, TWI_SET_DRIVE / TWI_READ_DRIVE)

- [ ] Channel 1: 0%, 50%, 100% - confirm PWM duty cycle and resulting drive
- [ ] Channel 2: 0%, 50%, 100% - confirm PWM duty cycle and resulting drive

Notes:

## 5. Readbacks (`GRFPPVP` / `GRFPPVN` / `GRFDRV` / `GRFPWR`)

- [X] RF+ level readback matches a known reference signal
- [X] RF- level readback matches a known reference signal
- [X] Power readback is sane at a known drive level

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

- [X] `TUNERFCH` on channel 1 completes and lands on a sensible frequency
- [X] `RETUNERFCH` on channel 1 (from a known-good frequency) converges
- [X] Repeat both on channel 2

Notes:

## 9. Persistence (`SAVE` / `RESTORE` / `FORMAT`)

- [X] `SAVE`, power-cycle, settings are restored on boot
- [X] `RESTORE` reloads the last saved settings
- [ ] `FORMAT`, confirm it resets to Rev 1 defaults

Notes:

## 10. Firmware field update over direct USB (`PGM` command) - test LAST, on a recoverable bench unit

**Passed on Rev 1.5, September 8, 2026** (results filled in below). Rev 1.4
failed this section outright - see the notes at the end. Re-run this whole
section against any change to `ProgramFLASHcmd()` or
`IAP_CopyStagingToAppAndReset()`, on a unit you're set up to recover - not a
unit that's already installed.

This section is direct-USB only - proving the on-module update mechanism
itself before it's relied on. The same thing through the MIPS TWI relay (so a
case never has to be opened at all) is §11, which also passes.

**Before starting**, confirm the recovery path works on this exact board:
double-tap its reset button, confirm a serial port appears (its SAM-BA
bootloader), and confirm `bossac` can reflash it from that state. Use
[firmware/RFdriver_v1.5.bin](firmware/RFdriver_v1.5.bin) (size 77984, CRC-8
106 - see [firmware/README.md](firmware/README.md)) as the known-good image.
Note that `pio run -t upload` is *not* a reliable recovery tool here: its
1200-baud touch-reset assumes the application is running, so it fails from an
already-bootloader state. Use bossac directly:

```
~/.platformio/packages/tool-bossac/bossac --port=cu.usbmodemXXXX \
    -e -w -v -R -o 0x2000 .pio/build/adafruit_feather_m0/firmware.bin
```

Use [pgm_update.py](pgm_update.py) to actually drive the transfers below - a
plain terminal can't type the hex-encoded image body. Requires `pip install
pyserial`. Examples:

```
python3 pgm_update.py --port <RFdriver's port> --check-only
python3 pgm_update.py --port <RFdriver's port> --file firmware/RFdriver_v1.5.bin --expect-version "1.5"
python3 pgm_update.py --port <RFdriver's port> --file firmware/RFdriver_v1.5.bin --crc 0
python3 pgm_update.py --port <RFdriver's port> --file firmware/RFdriver_v1.5.bin --truncate 40000
python3 pgm_update.py --port <RFdriver's port> --file README.md
```

To prove a real version change rather than a rewrite of identical bytes, send
a *different* image than the one running (e.g. v1.4 onto a v1.5 board) and
check `GVER` afterwards. Note v1.4 cannot update itself, so recover from it
with bossac.

- [x] Recovery path (above) confirmed working *before* attempting an update
      **PASS** - recovered cleanly from a hung state five times, plus once
      from a partially-written flash after a failed `pio` upload
- [x] `PGM,<size>` with a deliberately-bad file (a good image with a wrong
      trailing CRC, `--crc 0`): **PASS** - all 305 rows staged, then
      "CRC mismatch or malformed transfer - update aborted. The application
      partition was never touched", board still running v1.5 afterwards
- [x] `PGM,<size>` with a file that isn't a valid image at all (a text file):
      **PASS** - rejected at row 0 with "Image does not look like a valid
      firmware image for this board - aborted, nothing written."
- [x] `PGM,77984` with `firmware/RFdriver_v1.5.bin` and CRC `106`:
      **PASS** - transfer completed, board reported success, reset, and
      `GVER` afterwards reported "RFdriver version 1.5, September 8, 2026"
- [x] A genuine version change, not just a rewrite of identical bytes:
      **PASS** - board running v1.5 was sent `firmware/RFdriver_v1.4.bin`
      (77952 bytes, CRC 112) and came back reporting "RFdriver version 1.4,
      September 7, 2026". This is the test that actually proves the feature
      works; every other successful case above writes back content identical
      to what was already flashed. Recovered to v1.5 with `bossac` after.
- [x] Dropped mid-transfer (`--truncate 40000` against a declared 77984):
      **PASS** - board hit its own 10 s idle timeout, reported "Firmware
      update timed out - aborted. The application partition was never
      touched", and kept running
- [ ] RF drive on both channels reads 0 immediately once the transfer starts
      **NOT VERIFIABLE OVER SERIAL, needs a scope.** `UpdateCH1Drive(0)` /
      `UpdateCH2Drive(0)` do run before anything else and write the PWM duty
      register (`TCC2->CC[1]`) directly, so the RF output really is forced
      off. But `GRFDRV` reports `rfdriver.RFCD[].DriveLevel`, the *setpoint*,
      which is deliberately not changed - so it still reads 25.00 during and
      after an update attempt (confirmed). Also note that after a *rejected*
      update the 25 ms control loop resumes and restores the PWM from that
      setpoint (RFdriver.cpp:1019-1020), which is correct behaviour for a
      board returning to service, but means "drive stays off" is only true
      during the transfer. Confirm with a scope on the RF output.
- [x] Time the transfer for a realistic-sized image, note it below
      **~13 s** for the full 77984-byte image over direct USB (305 rows,
      ~40 ms/row). Rejections take the same ~13 s, since the whole image is
      staged before the CRC is checked.
- [x] If anything goes wrong: confirm the recovery path (above) still works
      **PASS**

Notes:

**Rev 1.4 FAILED this section outright; Rev 1.5 passes it.**

Rev 1.4's `PGM` hung the board while writing row 10 (`0x2A00`), every time,
requiring physical recovery. Root cause: it streamed the image straight over
the running application, and `ProgramFLASHcmd()` itself lives at
`0x292C-0x2CDB` with its erase/write sequence at `0x2ad0-0x2ae8`, inside row
10 - `FlashClass::erase()` wiped that row and then returned into it, so the
CPU fetched erased `0xFF` as an instruction and faulted with interrupts
already disabled. Rows 0-9 survived only by luck of layout. Ruled out along
the way: speed/overrun (a 3 s gap before row 10 changed nothing), cumulative
timing, worn flash (new chip), and NVM lock regions (16KB-aligned, nowhere
near `0x2A00`).

Rev 1.5 stages the image in a separate flash partition and only overwrites
the application from a RAM-resident copier at the very end - see the README's
"Firmware field update" section. Retested with `pgm_update.py` on
September 8, 2026; all results above are from that session.

Also learned: `pio run -t upload` does **not** work from an already-bootloader
state (its 1200-baud touch-reset assumes the app is running), and was also
seen to fail mid-write once, leaving flash partially programmed. Recover with
`bossac` directly - the working invocation is in the README.

## 11. Firmware field update through the MIPS TWI relay (`TWITALK`)

**PASSED September 8, 2026** - this is the goal the whole feature exists for:
updating a module's firmware without opening the enclosure. Same mechanism as
§10, but the bytes reach the module through a MIPS controller's `TWITALK`
relay instead of the module's own USB port.

Setup used: MIPS controller v1.264 on the host USB port, RFdriver on board 0
at TWI address 112, reached with `TWITALK,0,112`. The module's own USB was not
connected - everything below went through the relay.

```
python3 pgm_update.py --port <MIPS port> --twi 0,112 --check-only
python3 pgm_update.py --port <MIPS port> --twi 0,112 --file firmware/RFdriver_v1.5.bin -y
python3 pgm_update.py --port <MIPS port> --twi 0,112 --file firmware/RFdriver_v1.5.bin --crc 0 -y
```

- [x] Tunnel opens and relays both directions: `GVER` through `TWITALK,0,112`
      returned "RFdriver version 1.5, September 8, 2026", and ESC closed it
      with "Exiting TWI redirection."
- [x] Full 77984-byte update through the relay: **PASS** - transfer completed,
      CRC verified, module committed, reset, and answered `GVER` again through
      a freshly-opened tunnel
- [x] Deliberately-bad CRC through the relay: **PASS** - all 305 rows relayed
      and staged, then "CRC mismatch or malformed transfer - update aborted.
      The application partition was never touched", module still running
      afterwards. This is the property that makes a relayed update safe to do
      on an installed module: a failure costs nothing.
- [x] Transfer time: **162.5 s** (2 m 43 s) for the full image, about 12.8x
      the ~13 s direct-USB time. The relay forwards host->slave one byte per
      I2C transaction with a `delay(1)` between each, so this is close to the
      floor for the current `twitalk()`.
- [x] MIPS watchdog survives a multi-minute relayed transfer (todo.md §3):
      **PASS** - two full 162 s transfers with no MIPS reset. Note this holds
      *because* the host waits for each row's "Next" before sending the next
      row. `twitalk()` only kicks the watchdog in its outer loop, and its
      inner drain loop runs `delay(1)` per byte with no kick - a host that
      streamed the image without pacing would sit in that loop for minutes and
      reset MIPS. **Do not "optimise" the per-row handshake away.**
- [x] The `'x'`/`0xFF` slave->host byte filter (todo.md §3): harmless, as
      predicted. "Next" arrives as "Net"; `pgm_update.py` treats any non-empty
      line as an ack. No message the firmware prints contains `'x'` or
      `0xFF` - worth preserving that when editing those strings.
- [x] `twitalk()` behaviour when the target resets mid-session (todo.md §3):
      the module resets on a successful update and stops answering, and MIPS
      stays in the relay loop until the host sends ESC. `pgm_update.py` always
      sends ESC in a `finally` block, and MIPS exited cleanly every time.
      After the reset the module is back in normal mode, so verifying the new
      version needs a *fresh* `TWITALK` - `--expect-version` cannot do this
      over the relay and is skipped there by design.

Notes:

One host-tool bug was found and fixed during this session: the `TWITALK`
banner is four lines printed with `delay(100)` between them, plus more setup
delay before the relay loop starts, and the tool's fixed-duration drain
returned as soon as the first bytes arrived - so the next command collided
with the tail of the banner. `Link.drain_until_quiet()` now waits for the line
to actually go idle. Anything else driving `TWITALK` should do the same.

## Overall result

- [ ] All sections pass
- [x] Issues found (list below, with section/item numbers)

- §10: Rev 1.4's `PGM` was fundamentally broken (erased the code it was
  executing; hung at row 10 every time). Fixed in Rev 1.5 by staging in a
  separate flash partition and committing from a RAM-resident copier. §10 and
  §11 both pass on 1.5.
- §10: the RF-drive-off check is not verifiable over serial - `GRFDRV` reports
  the setpoint, not the PWM state. Needs a scope. Still open.
- Sections 1, 3, 5 and 8 pass on Rev 1.5, and 2 and 9 pass except for the
  items noted below. **No issues found in any of them.**
- Still to do, none of them known problems - just not exercised yet:
  §2 extended-addressing jumpers, §4 drive level, §6 bug-fix regression
  checks, §7 gate input, §9 `FORMAT`.
