# TODO: firmware field update over TWI

The actual goal is updating RFdriver's firmware through the MIPS TWI bus, so
the enclosure never has to be opened at all - direct USB is only useful as a
stepping stone (proving the on-module mechanism works) and as the recovery
path if an update ever fails partway through. This file tracks what's left
to get from "works over USB" to "works over TWI, from the host app."

## 0. Prerequisite - DONE (Rev 1.5, September 8, 2026)

- [x] **`PGM` redesigned to stage in spare flash, then commit from RAM.**
      Rev 1.4 was fundamentally broken - it erased the flash rows holding the
      code it was executing and hard-hung the board at row 10 (`0x2A00`)
      every time. Rev 1.5 partitions flash (application `0x2000-0x21000`,
      staging `0x21000-0x40000`), receives into staging, re-reads and CRCs
      the staged image, and only then runs the RAM-resident
      `IAP_CopyStagingToAppAndReset()`. See the README's "Firmware field
      update" section and the Rev 1.5 entry in `src/RFdriver.cpp`.
- [x] [checklist.md](checklist.md) §10 passes over direct USB, including the
      deliberately-bad-file, dropped-transfer and recovery-path checks, and a
      genuine v1.5 -> v1.4 version change. Full 77984-byte round trip takes
      ~13 s.
- [x] [checklist.md](checklist.md) §11 passes through the MIPS TWI relay -
      the actual goal of this file. See §4 below.

(Two smaller items that were open here are now tracked in §5.)

## 1. Host-side upload path - SUPERSEDED, use `pgm_update.py`

~~Originally planned as a new feature inside `MIPSapp/MIPS_QT6`~~ - decided
against building this into the Qt app. [pgm_update.py](pgm_update.py) (added
alongside checklist.md item 10) already does the job and is a better fit:
this is a developer/field-update tool, not an end-user feature, so it
doesn't need to live inside the operator-facing app. It supports both
transports:

- Direct-USB (`pgm_update.py --file <img>`) - checklist.md §10, passing.
- TWI relay (`pgm_update.py --file <img> --twi <board>,<addr>` or `--wire1`)
  - opens the tunnel with `TWITALK`/`TWI1TALK`, runs the transfer as
    `"PGM," + size + "\n"` (no address argument, see README.md), and always
    sends ESC (`0x1B`) afterward, success or failure, so MIPS doesn't get
    left stuck relaying. Verified against real hardware - checklist.md §11,
    passing.

No UI entry point needed - free-text `--port`/`--twi board,addr` flags on
the command line are the whole interface, matching the "no dropdown, no
discovery UI" call already made when this was scoped as a Qt feature.

## 2. Decisions needed - ANSWERED

- [x] Acceptable transfer time. **Measured: 162.5 s (2 m 43 s)** for the full
      77984-byte image through the relay, versus ~13 s over direct USB - about
      12.8x. The relay forwards host->slave one byte per I2C transaction with a
      `delay(1)` between each, so that is close to the floor without changing
      `twitalk()` itself. Under three minutes for a field update that avoids
      opening an enclosure seems clearly acceptable; no further optimisation
      pursued.
- [ ] Whether an update should be blockable/disallowed while the system is
      actively running an experiment. Still open, and now better informed: the
      module holds RF drive off for the whole ~3 minutes, and the relay ties up
      the MIPS TWI bus and its host serial port for that time too. This is a
      policy call, not a technical one.

## 3. MIPS firmware (`platformIO/MIPS`) - VERIFIED, no changes needed

All three concerns were exercised by the §11 hardware run and none needs a fix.
See [checklist.md](checklist.md) §11 for the evidence.

- [x] The `'x'`/`0xFF` slave->host byte filter is harmless here. "Next" arrives
      as "Net"; the host treats any non-empty line as an ack. No message the
      RFdriver firmware prints contains `'x'` or `0xFF` - keep it that way when
      editing those strings.
- [x] The watchdog survives a multi-minute relayed transfer - two full 162 s
      transfers, no MIPS reset. **But only because the host waits for each
      row's "Next" before sending the next row.** `twitalk()` kicks the
      watchdog in its outer loop only, and its inner drain loop runs `delay(1)`
      per byte with no kick, so an unpaced host would sit in that loop for
      minutes and reset MIPS. The per-row handshake is load-bearing.
- [x] `twitalk()` when the target resets mid-session: MIPS stays in the relay
      loop polling a silent address until the host sends ESC, which
      `pgm_update.py` always does in a `finally` block. MIPS exited cleanly
      every time. After the reset the module is in normal mode, so verifying
      the new version needs a fresh `TWITALK`.

## 4. End-to-end validation - DONE

- [x] [checklist.md](checklist.md) §11 records a full relayed update against
      real hardware (MIPS v1.264, board 0, TWI address 112): tunnel open/close,
      a complete 77984-byte update with reset and version re-check, and a
      deliberately-bad-CRC run that was rejected with the module still running.

## 5. Remaining work

- [ ] Harden the NVM busy-waits. `FlashClass`'s
      `while (!NVMCTRL->INTFLAG.bit.READY) {}` never checks the lock/program
      error flags, so any NVM error hangs the board forever with interrupts off
      instead of failing with a NAK. Not the cause of the Rev 1.4 hang, but the
      same class of unrecoverable failure.
- [ ] Verify the RF-drive-off behaviour with a scope (checklist §10) - it
      cannot be checked over serial, since `GRFDRV` reports the setpoint rather
      than the PWM state.
- [ ] Decide the "block updates during an experiment?" policy question in §2.
