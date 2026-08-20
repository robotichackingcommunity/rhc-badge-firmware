#!/usr/bin/env python3
"""m3mdump.py -- test utility: dump MCU memory from the RHC badge via the hidden
`m3mdump` serial command and save it to a .bin file.

By default it dumps the STM32 internal flash: 0x08000000 + 128 KiB
(the STM32U073CBT6 has 128 KiB of flash).

Requirements:
  * pip install pyserial
  * The badge must be in **AI Interactive** mode (top menu item, press B3) --
    the firmware only services serial commands (m3mdump included) in that mode.

Usage:
  python m3mdump.py                                   # 0x08000000, 128 KiB -> flash_dump.bin
  python m3mdump.py --base 0x08000000 --size 0x20000 --out flash.bin
  python m3mdump.py --port COM7
  python m3mdump.py --base 0x20000000 --size 0x2000   # 8 KiB of SRAM
"""
import argparse
import os
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")

BAUD = int(os.environ.get("BADGE_BAUD", "115200"))
CHUNK = 4096                      # firmware caps a single m3mdump at 4096 bytes

# CH340 and friends (same table as badge.py).
BADGE_USB_IDS = {(0x1A86, 0x7523), (0x1A86, 0x7522),
                 (0x1A86, 0x5523), (0x1A86, 0x55D4)}
BADGE_DESC_HINTS = ("CH340", "CH9102", "USB-SERIAL", "USB SERIAL", "USB2.0-SERIAL")


def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid is not None and (p.vid, p.pid) in BADGE_USB_IDS:
            return p.device
    for p in ports:
        desc = f"{p.description or ''} {p.manufacturer or ''}".upper()
        if any(h in desc for h in BADGE_DESC_HINTS):
            return p.device
    for p in ports:
        if p.vid == 0x1A86:
            return p.device
    real = [p for p in ports if p.device
            and not p.device.startswith(("/dev/ttyS", "/dev/ttyAMA"))]
    return real[0].device if len(real) == 1 else None


def disable_hupcl(ser):
    """POSIX: stop closing the port from pulsing DTR and rebooting the CH340."""
    if sys.platform == "win32":
        return
    try:
        import termios
        fd = ser.fileno()
        a = termios.tcgetattr(fd)
        a[2] &= ~termios.HUPCL          # cflag &= ~HUPCL
        termios.tcsetattr(fd, termios.TCSANOW, a)
    except Exception:
        pass                            # best-effort


def open_serial(port):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.timeout = 0.1
    # Keep DTR/RTS low so opening the port doesn't trigger the CH340 auto-reset
    # (which would knock the badge out of AI mode). Windows doesn't pulse anyway.
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass
    ser.open()
    disable_hupcl(ser)
    time.sleep(0.2)
    return ser


def parse_line(line):
    """Parse one canonical hexdump line -> (addr, data, redacted_flags) or None.
    Redacted bytes appear as '--' and come back as 0x00 with the flag set."""
    if "|" not in line:
        return None
    hexpart = line.split("|", 1)[0]     # drop the ASCII sidebar
    toks = hexpart.split()
    if not toks or len(toks[0]) != 8:
        return None
    try:
        addr = int(toks[0], 16)
    except ValueError:
        return None
    data = bytearray()
    flags = []
    for t in toks[1:]:
        if t == "--":                   # firmware redacted this byte
            data.append(0)
            flags.append(True)
        else:
            try:
                data.append(int(t, 16))
            except ValueError:
                return None
            flags.append(False)
    return addr, bytes(data), flags


def drain(ser, secs=0.2):
    end = time.time() + secs
    while time.time() < end:
        if not ser.read(4096):
            break


def ping(ser):
    drain(ser)
    ser.write(b"ping\n")
    ser.flush()
    end = time.time() + 2.0
    buf = b""
    while time.time() < end:
        buf += ser.read(256)
        if b"pong" in buf:
            return True
    return False


def dump_chunk(ser, addr, length, redacted):
    drain(ser, 0.05)
    ser.write(f"m3mdump 0x{addr:08X} 0x{length:X}\n".encode())
    ser.flush()
    want = (length + 15) // 16
    got = {}
    buf = b""
    deadline = time.time() + 5.0
    while len(got) < want and time.time() < deadline:
        chunk = ser.read(4096)
        if not chunk:
            time.sleep(0.005)
            continue
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("latin-1", "replace").rstrip("\r")
            if not line:
                continue
            if line.startswith("m3mdump"):           # firmware usage/error message
                raise RuntimeError(f"badge said: {line!r}")
            p = parse_line(line)
            if p is None:
                continue
            a, d, flags = p
            got[a] = d
            for i, f in enumerate(flags):
                if f:
                    redacted.add(a + i)
            deadline = time.time() + 2.0              # extend while making progress
    if len(got) < want:
        raise TimeoutError(
            f"got {len(got)}/{want} lines at 0x{addr:08X} "
            f"(is the badge in AI Interactive mode?)")
    out = bytearray()
    a = addr
    while a < addr + length:
        d = got.get(a)
        if d is None:
            raise RuntimeError(f"missing hexdump line @ 0x{a:08X}")
        out += d
        a += 16
    return bytes(out[:length])


def human(n):
    if n < 1024:
        return f"{n} B"
    if n < 1024 * 1024:
        return f"{n / 1024:.0f} KiB"
    return f"{n / (1024 * 1024):.1f} MiB"


def main():
    ap = argparse.ArgumentParser(description="Dump RHC badge memory via m3mdump.")
    ap.add_argument("--port", help="serial port (auto-detect if omitted)")
    ap.add_argument("--base", default="0x08000000",
                    help="start address (default 0x08000000, flash base)")
    ap.add_argument("--size", default=str(128 * 1024),
                    help="bytes to dump (default 131072 = 128 KiB)")
    ap.add_argument("--out", default="flash_dump.bin", help="output .bin file")
    ap.add_argument("--no-ping", action="store_true",
                    help="skip the AI-mode health check")
    args = ap.parse_args()

    base = int(args.base, 0)
    size = int(args.size, 0)
    if size <= 0:
        sys.exit("--size must be positive")

    port = args.port or os.environ.get("BADGE_PORT") or find_port()
    if not port:
        sys.exit("Could not find the badge serial port. Pass --port COM7 / "
                 "/dev/ttyUSB0.")

    print(f"Port : {port} @ {BAUD}")
    print(f"Range: 0x{base:08X} .. 0x{base + size:08X}  "
          f"({size} bytes, {human(size)})")
    print(f"Out  : {args.out}")

    ser = open_serial(port)
    try:
        if not args.no_ping:
            if ping(ser):
                print("Badge responded to ping (AI Interactive mode OK).")
            else:
                print("WARNING: no ping reply -- put the badge in AI Interactive "
                      "mode (top menu, B3). Continuing anyway...")

        redacted = set()
        t0 = time.time()
        with open(args.out, "wb") as f:
            done = 0
            while done < size:
                n = min(CHUNK, size - done)
                data = dump_chunk(ser, base + done, n, redacted)
                f.write(data)
                done += n
                pct = 100 * done / size
                bar = int(pct // 4)
                sys.stdout.write(
                    f"\r[{'#' * bar}{'.' * (25 - bar)}] {pct:5.1f}%  "
                    f"{done}/{size} bytes")
                sys.stdout.flush()
        dt = time.time() - t0
        print(f"\nDone in {dt:.1f}s -> {args.out} "
              f"({os.path.getsize(args.out)} bytes)")
        if redacted:
            runs = []
            for a in sorted(redacted):
                if runs and a == runs[-1][1] + 1:
                    runs[-1][1] = a
                else:
                    runs.append([a, a])
            print(f"Redacted (returned 0x00) {len(redacted)} byte(s) the firmware "
                  f"refused to reveal (admin secret):")
            for lo, hi in runs:
                print(f"  0x{lo:08X}..0x{hi:08X}  ({hi - lo + 1} byte(s))")
    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
