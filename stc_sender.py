#!/usr/bin/env python3
"""
STC UART Sender (Arduino emulator)
----------------------------------
Emulates the Arduino's byte output using a USB-TTL converter.

Protocol:
  Single thruster STC nibble (low 4 bits of one byte):
    bit0 = direction (0/1)
    bits1..3 = magnitude (0..5 used; 6..7 reserved)
    Receiver may compute speed = 2 * magnitude

  Quad thruster word (optional --quad):
    uint16_t: [stc:TL][stc:TR][stc:BL][stc:BR]
    bit ranges: [0:3][4:7][8:11][12:15] (send low byte then high byte)

Install:
  pip install pyserial

Examples:
  # Fixed single-thruster: dir=1, mag=3, 8E1, 10 Hz
  ./stc_sender.py /dev/ttyUSB0 115200 --mode fixed --dir 1 --mag 3 --parity E --hz 10

  # Sweep mag 0..5 at 5 Hz, toggle dir every wrap
  ./stc_sender.py /dev/ttyUSB0 115200 --mode sweep --hz 5 --parity E

  # Interactive (type commands):  set dir 1 | set mag 4 | send | quad 1,3,9,D | quit
  ./stc_sender.py /dev/ttyUSB0 115200 --mode interactive --parity E

  # Quad mirror of one STC (same nibble to all four), 8N1
  ./stc_sender.py /dev/ttyUSB0 115200 --quad --mode fixed --dir 0 --mag 5 --parity N

  # Quad explicit nibbles (comma-separated hex 0..F): TL,TR,BL,BR
  ./stc_sender.py /dev/ttyUSB0 115200 --quad --mode fixed --nibbles C,D,9,1
"""
import sys
import time
import argparse

try:
    import serial  # pyserial
except ImportError:
    print("pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(1)

PARITY_MAP = {
    'N': serial.PARITY_NONE,
    'E': serial.PARITY_EVEN,
    'O': serial.PARITY_ODD,
    'M': serial.PARITY_MARK,
    'S': serial.PARITY_SPACE,
}
STOPBITS_MAP = {1: serial.STOPBITS_ONE, 2: serial.STOPBITS_TWO}
BYTESIZE_MAP = {5: serial.FIVEBITS, 6: serial.SIXBITS, 7: serial.SEVENBITS, 8: serial.EIGHTBITS}

def build_stc(direction: int, magnitude: int) -> int:
    d = 1 if direction else 0
    m = 0 if magnitude < 0 else (5 if magnitude > 5 else magnitude)
    return ((m & 0x07) << 1) | (d & 0x01)  # 0..15

def pack_quad(tl: int, tr: int, bl: int, br: int) -> int:
    # Nibbles should be 0..15 each
    w = 0
    w |= (tl & 0x0F) << 0
    w |= (tr & 0x0F) << 4
    w |= (bl & 0x0F) << 8
    w |= (br & 0x0F) << 12
    return w & 0xFFFF

def open_serial(port, baud, parity, bytesize, stopbits, rtscts, dsrdtr, timeout=1.0):
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=BYTESIZE_MAP[bytesize],
            parity=PARITY_MAP[parity],
            stopbits=STOPBITS_MAP[stopbits],
            timeout=timeout,
            rtscts=rtscts,
            dsrdtr=dsrdtr,
        )
        return ser
    except Exception as e:
        print(f"Failed to open {port}: {e}", file=sys.stderr)
        print("Tip: add your user to 'dialout' group on Linux, or check udev permissions.", file=sys.stderr)
        sys.exit(1)

def send_single(ser, stc: int):
    b = stc & 0x0F  # low nibble only
    ser.write(bytes([b]))

def send_quad(ser, word: int):
    lo = word & 0xFF
    hi = (word >> 8) & 0xFF
    ser.write(bytes([lo, hi]))

def parse_nibbles(s: str):
    # "C,D,9,1" -> [0xC, 0xD, 0x9, 0x1]
    parts = [p.strip() for p in s.split(',')]
    if len(parts) != 4:
        raise ValueError("Provide exactly 4 comma-separated hex nibbles for TL,TR,BL,BR.")
    vals = []
    for p in parts:
        if len(p) == 0:
            raise ValueError("Empty nibble in --nibbles.")
        v = int(p, 16)
        if not (0 <= v <= 0xF):
            raise ValueError("Nibble out of range 0x0..0xF.")
        vals.append(v)
    return vals  # [TL,TR,BL,BR]

def run_fixed(ser, quad, hz, direction, magnitude, nibbles):
    period = 1.0 / max(1.0, hz)
    if quad:
        if nibbles is not None:
            TL, TR, BL, BR = parse_nibbles(nibbles)
            word = pack_quad(TL, TR, BL, BR)
        else:
            stc = build_stc(direction, magnitude)
            word = pack_quad(stc, stc, stc, stc)
        while True:
            send_quad(ser, word)
            time.sleep(period)
    else:
        stc = build_stc(direction, magnitude)
        while True:
            send_single(ser, stc)
            time.sleep(period)

def run_sweep(ser, quad, hz, start_dir):
    period = 1.0 / max(1.0, hz)
    direction = 1 if start_dir else 0
    mag = 0
    while True:
        stc = build_stc(direction, mag)
        if quad:
            word = pack_quad(stc, stc, stc, stc)
            send_quad(ser, word)
        else:
            send_single(ser, stc)
        mag += 1
        if mag > 5:
            mag = 0
            direction ^= 1  # toggle direction at each wrap
        time.sleep(period)

def run_interactive(ser, quad):
    print("Interactive mode. Commands:")
    print("  set dir <0|1>")
    print("  set mag <0..5>")
    print("  send                (single)")
    print("  quad <TL,TR,BL,BR> (hex nibbles, e.g., C,D,9,1)")
    print("  mirror              (mirror current stc to all four and send)")
    print("  burst <N> <Hz>      (send N frames at given Hz)")
    print("  show                (print current stc)")
    print("  quit/exit")
    direction, magnitude = 0, 0
    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not cmd:
            continue
        parts = cmd.split()
        try:
            if parts[0] == "set" and len(parts) >= 3:
                if parts[1] == "dir":
                    direction = 1 if int(parts[2]) else 0
                elif parts[1] == "mag":
                    magnitude = max(0, min(5, int(parts[2])))
                else:
                    print("Unknown field. Use: set dir <0|1> | set mag <0..5>")
            elif parts[0] == "send":
                stc = build_stc(direction, magnitude)
                if quad:
                    word = pack_quad(stc, stc, stc, stc)
                    send_quad(ser, word)
                else:
                    send_single(ser, stc)
            elif parts[0] == "quad" and len(parts) >= 2:
                TL, TR, BL, BR = parse_nibbles(parts[1])
                word = pack_quad(TL, TR, BL, BR)
                send_quad(ser, word)
            elif parts[0] == "mirror":
                stc = build_stc(direction, magnitude)
                word = pack_quad(stc, stc, stc, stc)
                send_quad(ser, word)
            elif parts[0] == "burst" and len(parts) >= 3:
                N = max(1, int(parts[1]))
                hz = float(parts[2])
                period = 1.0 / max(1.0, hz)
                stc = build_stc(direction, magnitude)
                for _ in range(N):
                    if quad:
                        word = pack_quad(stc, stc, stc, stc)
                        send_quad(ser, word)
                    else:
                        send_single(ser, stc)
                    time.sleep(period)
            elif parts[0] == "show":
                stc = build_stc(direction, magnitude)
                print(f"dir={direction} mag={magnitude} stc=0x{stc:X} ({stc:04b})")
            elif parts[0] in ("quit", "exit"):
                break
            else:
                print("Unknown command.")
        except Exception as e:
            print(f"Error: {e}")

def main():
    ap = argparse.ArgumentParser(description="STC UART sender (Arduino emulator)")
    ap.add_argument('port', nargs='?', default='/dev/ttyUSB0',
                    help='Serial port (e.g., /dev/ttyUSB0, /dev/ttyACM0, COM5)')
    ap.add_argument('baud', nargs='?', default=115200, type=int,
                    help='Baud rate (default 115200)')
    ap.add_argument('--quad', action='store_true',
                    help='Send 2-byte quad word [TL|TR|BL|BR] (low byte, high byte)')
    ap.add_argument('--mode', choices=['fixed','sweep','interactive'], default='sweep',
                    help='fixed: constant STC; sweep: mag 0..5, toggle dir each wrap; interactive: CLI')
    ap.add_argument('--dir', type=int, default=0, help='Direction for fixed mode (0 or 1)')
    ap.add_argument('--mag', type=int, default=0, help='Magnitude for fixed mode (0..5)')
    ap.add_argument('--nibbles', type=str, default=None,
                    help='Quad-only: TL,TR,BL,BR as hex nibbles (e.g., "C,D,9,1")')
    ap.add_argument('--hz', type=float, default=5.0,
                    help='Send frequency (frames per second) for fixed/sweep (default 5)')
    ap.add_argument('--parity', choices=['N','E','O','M','S'], default='E',
                    help='UART parity (default E = even)')
    ap.add_argument('--bytesize', type=int, choices=[5,6,7,8], default=8,
                    help='UART data bits (default 8)')
    ap.add_argument('--stopbits', type=int, choices=[1,2], default=1,
                    help='UART stop bits (default 1)')
    ap.add_argument('--rtscts', action='store_true', help='Enable RTS/CTS hardware flow control')
    ap.add_argument('--dsrdtr', action='store_true', help='Enable DSR/DTR hardware flow control')
    args = ap.parse_args()

    ser = open_serial(args.port, args.baud, args.parity, args.bytesize,
                      args.stopbits, args.rtscts, args.dsrdtr)

    print(f"Opened {args.port} @ {args.baud} "
          f"({args.bytesize}{args.parity}{args.stopbits}, rtscts={args.rtscts}, dsrdtr={args.dsrdtr}) "
          f"mode={args.mode} quad={args.quad}")
    try:
        if args.mode == 'fixed':
            run_fixed(ser, args.quad, args.hz, args.dir, args.mag, args.nibbles)
        elif args.mode == 'sweep':
            run_sweep(ser, args.quad, args.hz, args.dir)
        else:
            run_interactive(ser, args.quad)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == '__main__':
    main()
