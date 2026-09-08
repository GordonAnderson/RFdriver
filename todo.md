# TODO: firmware field update over TWI

The actual goal is updating RFdriver's firmware through the MIPS TWI bus, so
the enclosure never has to be opened at all - direct USB is only useful as a
stepping stone (proving the on-module mechanism works) and as the recovery
path if an update ever fails partway through. This file tracks what's left
to get from "works over USB" to "works over TWI, from the host app."

## 0. Prerequisite - do not start on anything below until this passes

- [ ] [checklist.md](checklist.md) §10 (firmware update over direct USB)
      passes in full, on a bench unit, including the deliberately-bad-file
      and recovery-path checks. The TWI relay adds a slower, less reliable
      transport on top of the same mechanism - it's not worth building until
      the mechanism itself is proven.

## 1. MIPS host app (`MIPSapp/MIPS_QT6`) - new upload path

Nothing here exists yet; `TWITALK` (the relay command) is never called
anywhere in the Qt app today.

- [ ] New `Comms` method, mirroring `Comms::ARBupload()` in `comms.cpp`
      (address+size header, hex-encoded body in chunks, CRC trailer) but:
      1. First opens the tunnel: `SendCommand("TWITALK," + board + "," +
         twiAddr + "\n")` (or `TWI1TALK` if the target is on the `Wire1` bus)
      2. Runs the transfer as `"PGM," + size + "\n"` instead of
         `"ARBPGM," + addr + "," + size + "\n"` - no address argument, see
         [README.md](README.md)
      3. Always closes the tunnel afterward by sending ESC (`0x1B`), success
         or failure, so MIPS doesn't get left stuck relaying
- [ ] UI entry point: free-text board/address prompt + file picker, matching
      the existing ARB upload action in `mips.cpp` / `fileops.cpp` and the
      `PutEEPROM()` convention (`Board`, `TWI address`) already used
      elsewhere. **Decided: no dropdown, no discovery UI** - this is a
      developer tool for field updates, not an end-user feature, so
      free-text board+address input is the whole UI; it does not need to be
      friendlier than that.

## 2. Decisions needed (not mine to make)

- [ ] Acceptable transfer time. The relay forwards host->slave bytes one at a
      time, one I2C transaction per byte with a 1ms delay between each
      (`twitalk()` in the MIPS firmware) - for an ~80KB image at 2 hex
      characters per byte that's on the order of many minutes, unmeasured.
      Worth benchmarking directly-over-USB first (checklist §10 records this)
      to get a real baseline, then estimating the TWI multiplier before
      deciding this is acceptable as-is
- [ ] Whether an update should be blockable/disallowed while the system is
      actively running an experiment, given item 3's system-wide freeze

## 3. MIPS firmware (`platformIO/MIPS`) - things to verify, not necessarily fix

- [ ] `twitalk()`'s slave->host relay drops any byte equal to 120 (`'x'`) or
      255 (`0xFF`) before forwarding it to the host (see `Serial.cpp` in the
      MIPS firmware). This silently corrupts RFdriver's own `"Next"` progress
      message (it contains an `'x'`) when relayed. Traced through
      `Comms::ARBupload()`'s `getline()`/`waitforline()` logic and confirmed
      this is harmless for that specific check (it only tests for a
      non-empty line, not its content) - but it's a real, existing quirk of
      shared infrastructure, not something to silently patch without
      understanding why the filter is there for other module types. Flag it
      if a future protocol change ever needs the exact content of a relayed
      line to matter.
- [ ] Confirm the watchdog (`WDT_Restart(WDT)`, called once per `twitalk()`
      loop iteration) doesn't trip during a multi-minute relayed transfer -
      should be fine given how tight that loop is, but unverified over a
      real multi-minute run.
- [ ] Confirm `twitalk()` behaves correctly if the *target* module resets
      mid-session (which is exactly what a successful `PGM` does on
      purpose) - does MIPS notice the TWI address stop responding and exit
      cleanly, or does it hang until the host sends ESC? A successful update
      ends with RFdriver resetting itself; the host-side code from item 1
      needs to know to send ESC (or just stop) at that point rather than
      wait for a response that will never come from mid-reset silicon.

## 4. End-to-end validation, once 1-3 are done

- [ ] New checklist section (in this file's companion `checklist.md`, or a
      MIPS-side equivalent) mirroring §10 but exercised through `TWITALK`
      from the actual MIPS controller - same recovery-first discipline:
      confirm direct-USB recovery still works before trusting the relayed
      path with a unit that matters.
