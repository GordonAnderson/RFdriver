#!/usr/bin/env python3
"""
pgm_update.py - host-side driver for RFdriver's PGM firmware-update command.

This is the developer tool checklist.md item 10 calls for: something that can
actually drive the PGM transfer protocol end to end (a plain terminal can't -
the hex-encoded image body has no delimiters a human could type). It speaks
the exact protocol ProgramFLASHcmd() implements in src/Hardware.cpp, which is
also the protocol Comms::ARBupload() in the MIPS Qt app already speaks for
other modules' FLASH (see README.md's "Firmware field update" section):

    PGM,<size>\\n               <size> = image size in decimal bytes
    <hex bytes>                 the image, ASCII hex, two chars per byte,
                                 no separators
    \\n<crc>\\n                  the image's 8-bit CRC (poly 0x1D), decimal

Two transports:
  - direct (default): talk straight to RFdriver's own USB-CDC serial port.
    This is checklist.md item 10, and is verified working against firmware
    1.5 (full 77984-byte round trip, ~13 s). Firmware 1.4's PGM is broken and
    will hang the board - see the README.
  - --twi: relay through a MIPS controller's TWITALK/TWI1TALK tunnel to an
    RFdriver on the TWI bus (see twitalk() in the MIPS firmware's
    src/Serial.cpp). This is the "next level" from todo.md - do NOT trust it
    with a unit that matters until direct-USB (checklist.md item 10) passes
    in full, per todo.md item 0. See the docstring on twi_open()/twi_close()
    below for what's still unverified about it.

Usage examples:
    # Sanity-check you're talking to the right board before doing anything else
    python3 pgm_update.py --port /dev/tty.usbmodem1234 --check-only

    # The real transfer test (checklist.md item 10, bullet 3)
    python3 pgm_update.py --port /dev/tty.usbmodem1234 \\
        --file firmware/RFdriver_v1.5.bin --expect-version "1.5"

    # Deliberately-bad-file tests (checklist.md item 10, bullet 2)
    python3 pgm_update.py --port /dev/tty.usbmodem1234 \\
        --file firmware/RFdriver_v1.5.bin --crc 0        # wrong trailing CRC
    python3 pgm_update.py --port /dev/tty.usbmodem1234 \\
        --file firmware/RFdriver_v1.5.bin --truncate 40000  # dropped mid-transfer
    python3 pgm_update.py --port /dev/tty.usbmodem1234 \\
        --file README.md                                 # not an image at all

    # Through a MIPS controller's TWI relay, once that's worth trying (todo.md)
    python3 pgm_update.py --port /dev/tty.usbmodemMIPS1 \\
        --file firmware/RFdriver_v1.5.bin --twi 1,0x50

Needs pyserial (`pip install pyserial`).
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pgm_update.py needs pyserial - try: pip install pyserial")


# --- Protocol constants, mirrored from the firmware/host sources below ---
# ROW_SIZE:   FLASH_ROW_SIZE, include/Hardware.h - the chip's NVM erase/write
#             granularity, and the pacing unit ProgramFLASHcmd() acks on.
# CRC_POLY:   ComputeCRCbyte()/ComputeCRC() in src/Hardware.cpp, and
#             Comms::CalculateCRC() in the MIPS host app's comms.cpp - all
#             three implement the same CRC-8, so a value computed by any one
#             of them should agree with the others.
ACK = 0x06
NAK = 0x15
ROW_SIZE = 256
CRC_POLY = 0x1D

# ProgramFLASHcmd() times out any single idle byte-wait at 10s (see the
# `millis() > start + 10000` checks in src/Hardware.cpp). Our own waits are
# set comfortably longer than that so we see the board's own timeout message
# rather than racing it with one of our own.
ROW_WAIT_TIMEOUT = 15.0
PARTIAL_ROW_SETTLE = 1.5  # a trailing partial row is not acked (neither the
                          # firmware nor Comms::ARBupload() expects it to be) -
                          # just a brief window to catch a rejection.
FINAL_WAIT_TIMEOUT = 20.0

# Substrings ProgramFLASHcmd() actually prints on a rejected transfer (see
# src/Hardware.cpp). Matched case-insensitively. None of these contain 'x' or
# raw 0xFF, so they survive the MIPS TWI relay's slave->host byte filter
# (twitalk() in the MIPS firmware drops bytes 120/'x' and 255 - see todo.md
# item 3) even though "Next" itself does not - that's why the per-row wait
# below treats any non-empty line as an ack rather than matching "Next"
# literally.
FAILURE_PHRASES = (
    "does not look like a valid firmware image",
    "flash verify error",
    "failed its read-back crc",
    "crc mismatch",
    "malformed transfer",
    "firmware update timed out",
)

# The line ProgramFLASHcmd() prints immediately before it commits and resets.
# Deliberately matched on the LEADING words of that message, not the tail: the
# board resets ~50ms after printing it, and over the MIPS TWI relay the host
# has to poll the text out of the module's 512-byte SerialBuffer 30 bytes at a
# time (requestEventProcessor() in RFdriver.cpp). A tail match could be lost to
# a truncated read and turn a successful update into a reported failure. These
# words land inside the first 30-byte poll.
SUCCESS_PHRASE = "image received and verified"


def crc8(data):
    """Port of ComputeCRCbyte()/Comms::CalculateCRC() - CRC-8, poly 0x1D, run
    byte-by-byte over the raw (decoded) image bytes, matching what
    ProgramFLASHcmd() accumulates as it decodes each incoming hex pair."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ CRC_POLY) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


class Link:
    """Byte/line-oriented reader over a pyserial port, with its own small
    receive buffer so byte-level reads (ACK/NAK) and line-level reads
    ("Next", error/status messages) can be freely mixed."""

    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()

    def _fill(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
                return True
            time.sleep(0.01)
        return False

    def read_byte(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            if self.buf:
                b = self.buf[0]
                del self.buf[0:1]
                return b
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._fill(min(remaining, 0.2))

    def read_line(self, timeout):
        """Returns the next '\\n'-terminated line (stripped of \\r\\n), or
        None on timeout. Never returns an empty string - some of RFdriver's
        error messages start with an embedded '\\n' (see FAILURE_PHRASES
        callers in src/Hardware.cpp), which would otherwise show up as a
        spurious blank line ahead of the real message."""
        deadline = time.monotonic() + timeout
        while True:
            nl = self.buf.find(b"\n")
            if nl != -1:
                raw = bytes(self.buf[:nl])
                del self.buf[: nl + 1]
                line = raw.decode(errors="replace").strip("\r")
                if line != "":
                    return line
                continue
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._fill(min(remaining, 0.2))

    def read_ack(self, timeout):
        """Reads raw bytes until an ACK (0x06) or NAK (0x15) control byte is
        seen. Returns ("ACK"|"NAK"|None, raw bytes collected)."""
        deadline = time.monotonic() + timeout
        collected = bytearray()
        while time.monotonic() < deadline:
            b = self.read_byte(max(0.01, deadline - time.monotonic()))
            if b is None:
                break
            collected.append(b)
            if b == ACK:
                return "ACK", bytes(collected)
            if b == NAK:
                return "NAK", bytes(collected)
        return None, bytes(collected)

    def drain(self, duration):
        """Non-blocking-ish: soaks up whatever arrives in `duration` seconds
        and returns it decoded. Used to flush TWITALK's banner text."""
        self._fill(duration)
        s = bytes(self.buf)
        self.buf.clear()
        return s.decode(errors="replace")

    def drain_until_quiet(self, quiet_for=0.4, max_total=4.0):
        """Soaks up input until nothing new has arrived for `quiet_for`
        seconds. Needed for TWITALK's banner, which MIPS prints as four
        separate lines with delay(100) between them and then another delay
        before the relay loop starts - a fixed-duration drain either cuts the
        banner off (and the next command collides with its tail) or wastes
        time. Returns everything soaked up, decoded."""
        deadline = time.monotonic() + max_total
        collected = bytearray()
        while time.monotonic() < deadline:
            before = len(self.buf)
            self._fill(quiet_for)
            if len(self.buf) == before:
                break           # nothing new in the quiet window - done
            collected += self.buf
            self.buf.clear()
        collected += self.buf
        self.buf.clear()
        return bytes(collected).decode(errors="replace")

    def write(self, data):
        if isinstance(data, str):
            data = data.encode()
        self.ser.write(data)
        self.ser.flush()


def twi_open(link, board, addr, wire1):
    """Opens a TWITALK/TWI1TALK tunnel through a MIPS controller to the
    RFdriver at TWI address `addr` on board `board` (see twitalk() in the
    MIPS firmware's src/Serial.cpp) - everything sent after this is relayed
    byte-for-byte over TWI to that address, and its replies relayed back.

    Unverified per todo.md item 3 at the time this was written: whether the
    watchdog survives a multi-minute relayed transfer, and whether MIPS
    notices cleanly when RFdriver resets mid-session (a successful PGM
    transfer resets RFdriver on purpose). Don't be surprised if this needs
    follow-up once it's actually exercised end to end.
    """
    cmd = "TWI1TALK" if wire1 else "TWITALK"
    link.write(f"{cmd},{board},{addr}\n")
    # Wait for the banner to finish AND the relay loop to actually start -
    # anything sent before that collides with the tail of the banner.
    banner = link.drain_until_quiet()
    for line in banner.splitlines():
        if line.strip():
            print(f"[twi] {line.strip()}")


def twi_close(link):
    """Always send ESC to close the tunnel, success or failure, so MIPS
    doesn't get left stuck relaying (see todo.md item 1, bullet 3)."""
    try:
        link.write(b"\x1b")
        time.sleep(0.2)
        trailer = link.drain(0.3)
        if trailer.strip():
            print(f"[twi] {trailer.strip()}")
    except Exception as e:
        print(f"[twi] warning: error closing tunnel: {e}")


def run_check(link, label="board"):
    """Sends GVER and prints whatever comes back - a quick sanity check that
    we're talking to the right unit before risking an update on it."""
    link.buf.clear()
    link.write("GVER\n")
    line = link.read_line(3.0)
    if line is None:
        print(f"{label}: no response to GVER")
        return None
    print(f"{label}: {line}")
    return line


def run_transfer(link, data, declared_size, crc_value, truncate, quiet, pause_before_row=None):
    """Drives one PGM transfer. `data` is the (possibly --flip-byte modified)
    image bytes actually available to send; `declared_size` is what's sent
    in the PGM,<size> command line, which may deliberately differ from
    len(data) (see --truncate). `pause_before_row` is an optional {row: secs}
    map - PGM can't be paused/resumed across separate commands, but this
    lets a diagnostic run insert a dead-time gap before a specific row
    *within* one continuous session, to test an overrun/timing theory
    without ever needing to touch a firmware-side start address. Returns
    True only on a confirmed successful, verified update."""
    pause_before_row = pause_before_row or {}
    t0 = time.monotonic()
    link.buf.clear()

    print(f"-> PGM,{declared_size}")
    link.write(f"PGM,{declared_size}\n")
    status, raw = link.read_ack(5.0)
    if status != "ACK":
        extra = raw.decode(errors="replace").strip()
        print(f"Rejected at PGM,<size>: {status or 'no response'}" + (f" ({extra})" if extra else ""))
        return False
    print("ACK - board accepted the size, sending image...")

    hexdata = data.hex()
    num_full_rows = len(data) // ROW_SIZE
    remainder = len(data) % ROW_SIZE
    total_rows = num_full_rows + (1 if remainder else 0)

    for row in range(num_full_rows):
        if row in pause_before_row:
            secs = pause_before_row[row]
            print(f"  (pausing {secs:.1f}s before row {row}...)")
            time.sleep(secs)
        chunk = hexdata[row * ROW_SIZE * 2 : (row + 1) * ROW_SIZE * 2]
        link.write(chunk)
        # Every full row is acked with "Next", row 0 included (as of firmware
        # 1.5 - 1.4 held row 0 back and stayed silent for it). An implausible
        # vector table is rejected in place of row 0's ack.
        line = link.read_line(ROW_WAIT_TIMEOUT)
        if line is None:
            print(f"Timed out waiting for row {row} acknowledgment")
            return False
        if any(p in line.lower() for p in FAILURE_PHRASES):
            print(f"Rejected at row {row}: {line}")
            return False
        if not quiet:
            pct = 100.0 * ((row + 1) * ROW_SIZE) / len(data)
            elapsed = time.monotonic() - t0
            print(f"  row {row + 1}/{total_rows}  {pct:5.1f}%  {elapsed:6.1f}s  ({line!r})")

    if remainder:
        chunk = hexdata[num_full_rows * ROW_SIZE * 2 :]
        link.write(chunk)
        leftover = link.read_line(PARTIAL_ROW_SETTLE)
        if leftover and any(p in leftover.lower() for p in FAILURE_PHRASES):
            print(f"Rejected on final partial row: {leftover}")
            return False

    if truncate is not None:
        # Simulating a dropped connection: we've sent fewer bytes than
        # declared_size and stop here on purpose, without sending the
        # trailer. The board should sit waiting for the next hex pair and
        # hit its own 10s idle timeout (TimeoutExit in ProgramFLASHcmd()).
        print(f"--truncate given: stopped after {len(data)} of {declared_size} declared bytes, "
              f"no trailer sent - waiting to see the board time out...")
        line = link.read_line(FINAL_WAIT_TIMEOUT)
        elapsed = time.monotonic() - t0
        if line is None:
            print(f"No response within {FINAL_WAIT_TIMEOUT:.0f}s ({elapsed:.1f}s elapsed) - "
                  f"board may still be waiting on its own 10s timeout, or hung.")
        else:
            print(f"<- {line}  ({elapsed:.1f}s elapsed)")
        return False

    trailer = f"\n{crc_value}\n"
    link.write(trailer)
    print(f"-> trailer: CRC {crc_value}")

    line = link.read_line(FINAL_WAIT_TIMEOUT)
    elapsed = time.monotonic() - t0
    if line is None:
        print(f"Timed out waiting for final result ({elapsed:.1f}s elapsed)")
        return False
    print(f"<- {line}")
    ok = SUCCESS_PHRASE in line.lower()
    print(f"{'SUCCESS' if ok else 'REJECTED'} - transfer took {elapsed:.1f}s")
    return ok


def build_arg_parser():
    p = argparse.ArgumentParser(
        description="Drive RFdriver's PGM firmware-update command (checklist.md item 10).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--port", required=True, help="Serial device (RFdriver's own USB-CDC port in "
                                                   "direct mode; the MIPS controller's port in --twi mode)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--file", help="Firmware image to send (or any file, to test rejection of a "
                                   "non-image). Required unless --check-only.")
    p.add_argument("--size", type=int, default=None,
                    help="Override the declared size in PGM,<size>. Default: the file's actual size.")
    p.add_argument("--crc", type=int, default=None,
                    help="Override the trailing CRC value sent (default: correctly computed over "
                         "the data actually sent). Use a wrong value here to exercise the "
                         "CRC-mismatch rejection path (checklist.md's 'flip a byte in the CRC line').")
    p.add_argument("--flip-byte", type=int, action="append", default=[], metavar="OFFSET",
                    help="Invert all bits of the byte at OFFSET before sending (repeatable). "
                         "Combine with --crc set to a known-good value to exercise a CRC mismatch "
                         "against otherwise-plausible data, or leave --crc unset to see whether "
                         "the corrupted byte alone gets caught (e.g. offset < 256 hits the vector "
                         "table plausibility check).")
    p.add_argument("--truncate", type=int, default=None, metavar="N",
                    help="Only send the first N bytes of data, then stop without sending the "
                         "trailer - simulates a dropped connection / truncated transfer. The "
                         "declared size (--size, default the full file size) is sent unchanged, "
                         "so the board is left waiting and should hit its own timeout.")
    p.add_argument("--twi", metavar="BOARD,ADDR",
                    help="Relay through a MIPS controller's TWITALK tunnel instead of talking "
                         "direct-USB to RFdriver. See the twi_open() docstring before trusting "
                         "this with a unit that matters.")
    p.add_argument("--wire1", action="store_true", help="Use TWI1TALK instead of TWITALK (only with --twi)")
    p.add_argument("--check-only", action="store_true", help="Just send GVER and exit - no transfer.")
    p.add_argument("--expect-version", metavar="STR",
                    help="After a successful update, reconnect and confirm GVER's response "
                         "contains this substring.")
    p.add_argument("--pause-before-row", action="append", default=[], metavar="ROW:SECONDS",
                    help="Sleep SECONDS immediately before sending row-index ROW's chunk "
                         "(0-based, repeatable) - a same-session way to test an overrun/timing "
                         "theory around a specific row without a firmware change.")
    p.add_argument("-y", "--yes", action="store_true", help="Skip the confirmation prompt.")
    p.add_argument("-q", "--quiet", action="store_true", help="Suppress per-row progress lines.")
    return p


def main():
    args = build_arg_parser().parse_args()
    if not args.check_only and not args.file:
        sys.exit("--file is required unless --check-only is given")

    twi_board = twi_addr = None
    if args.twi:
        parts = args.twi.split(",")
        if len(parts) != 2:
            sys.exit("--twi wants BOARD,ADDR, e.g. --twi 1,0x50")
        twi_board, twi_addr = parts[0].strip(), parts[1].strip()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0)
    except serial.SerialException as e:
        sys.exit(f"Could not open {args.port}: {e}")
    link = Link(ser)

    tunnel_open = False
    try:
        if args.twi:
            twi_open(link, twi_board, twi_addr, args.wire1)
            tunnel_open = True

        if args.check_only:
            run_check(link)
            return

        with open(args.file, "rb") as f:
            data = bytearray(f.read())

        declared_size = args.size if args.size is not None else len(data)

        for offset in args.flip_byte:
            if not (0 <= offset < len(data)):
                sys.exit(f"--flip-byte {offset} is out of range for a {len(data)}-byte file")
            data[offset] ^= 0xFF
            print(f"flipped byte at offset {offset}: now 0x{data[offset]:02x}")

        send_data = bytes(data)
        if args.truncate is not None:
            if not (0 <= args.truncate <= len(send_data)):
                sys.exit(f"--truncate {args.truncate} is out of range for a {len(send_data)}-byte file")
            send_data = send_data[: args.truncate]

        crc_value = args.crc if args.crc is not None else crc8(send_data if args.truncate is None else data)

        print(f"file:     {args.file}")
        print(f"size:     {len(data)} bytes on disk, declaring {declared_size}, "
              f"sending {len(send_data)}")
        print(f"crc:      {crc_value}" + (" (overridden)" if args.crc is not None else " (computed)"))
        print(f"target:   {args.port}" + (f" -> TWI board {twi_board} addr {twi_addr}" if args.twi else " (direct)"))

        run_check(link, label="pre-flight GVER")

        if not args.yes:
            resp = input("Proceed with PGM transfer? [y/N] ").strip().lower()
            if resp != "y":
                print("Aborted.")
                return

        pause_map = {}
        for spec in args.pause_before_row:
            try:
                row_str, secs_str = spec.split(":")
                pause_map[int(row_str)] = float(secs_str)
            except ValueError:
                sys.exit(f"--pause-before-row wants ROW:SECONDS, e.g. --pause-before-row 10:2.0 (got {spec!r})")

        ok = run_transfer(link, send_data, declared_size, crc_value, args.truncate, args.quiet, pause_map)

        if ok and args.expect_version and args.twi:
            print("Update succeeded over the TWI relay - not attempting the --expect-version "
                  "reconnect (RFdriver just reset mid-tunnel; whether MIPS's twitalk() notices "
                  "cleanly is an open question, see todo.md item 3). Verify GVER by hand.")
        elif ok and args.expect_version:
            print("Update succeeded - board is resetting, reconnecting to verify...")
            time.sleep(2.0)
            ser.close()
            reopened = False
            for attempt in range(10):
                time.sleep(1.0)
                try:
                    ser2 = serial.Serial(args.port, args.baud, timeout=0)
                    reopened = True
                    break
                except serial.SerialException:
                    continue
            if not reopened:
                print(f"Could not reopen {args.port} after reset - check it manually.")
            else:
                link2 = Link(ser2)
                time.sleep(0.5)
                version_line = run_check(link2, label="post-update GVER")
                ser2.close()
                if version_line and args.expect_version in version_line:
                    print(f"PASS - version string contains '{args.expect_version}'")
                else:
                    print(f"FAIL - expected '{args.expect_version}' in GVER response")
    finally:
        if tunnel_open:
            twi_close(link)
        if ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
