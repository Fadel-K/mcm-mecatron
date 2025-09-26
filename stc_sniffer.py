#!/usr/bin/env python3
"""
STC UART Sniffer (Linux-friendly)
---------------------------------
Usage examples:
  ./stc_sniffer.py /dev/ttyUSB0 115200 --parity E --bytesize 8 --stopbits 1
  ./stc_sniffer.py /dev/ttyUSB0 115200 --quad --parity E
  ./stc_sniffer.py /dev/ttyACM0 115200 --rtscts

Notes:
  - Requires: pip install pyserial
  - Parity options: N (none), E (even), O (odd), M (mark), S (space)
  - In --quad mode: reads 2 bytes and decodes 4 STC nibbles:
    [stc:TL][stc:TR][stc:BL][stc:BR] mapped to bit ranges [0:3][4:7][8:11][12:15].
"""
import sys
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

STOPBITS_MAP = {
    1: serial.STOPBITS_ONE,
    2: serial.STOPBITS_TWO,
}

BYTESIZE_MAP = {
    5: serial.FIVEBITS,
    6: serial.SIXBITS,
    7: serial.SEVENBITS,
    8: serial.EIGHTBITS,
}

def bits8(b: int) -> str:
    return ''.join(str((b >> i) & 1) for i in range(7, -1, -1))

def decode_stc(nib: int):
    nib &= 0x0F
    dir_bit = nib & 0x01
    mag = (nib >> 1) & 0x07
    speed = 2 * mag
    return dir_bit, mag, speed

def open_serial(port, baud, parity, bytesize, stopbits, rtscts, dsrdtr, timeout=1):
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
        print("Hint: On Linux, ensure your user is in the 'dialout' group or check udev permissions.", file=sys.stderr)
        sys.exit(1)

def main():
    ap = argparse.ArgumentParser(description="STC UART sniffer")
    ap.add_argument('port', nargs='?', default='/dev/ttyUSB0',
                    help='Serial port (e.g., /dev/ttyUSB0, /dev/ttyACM0)')
    ap.add_argument('baud', nargs='?', default=115200, type=int,
                    help='Baud rate (default 115200)')
    ap.add_argument('--quad', action='store_true',
                    help='Read 2 bytes and decode 4 STC nibbles (TL,TR,BL,BR)')
    ap.add_argument('--parity', choices=list(PARITY_MAP.keys()), default='E',
                    help='UART parity (default E = even)')
    ap.add_argument('--bytesize', type=int, choices=[5,6,7,8], default=8,
                    help='UART data bits (default 8)')
    ap.add_argument('--stopbits', type=int, choices=[1,2], default=1,
                    help='UART stop bits (default 1)')
    ap.add_argument('--rtscts', action='store_true',
                    help='Enable RTS/CTS hardware flow control')
    ap.add_argument('--dsrdtr', action='store_true',
                    help='Enable DSR/DTR hardware flow control')
    args = ap.parse_args()

    ser = open_serial(args.port, args.baud, args.parity,
                      args.bytesize, args.stopbits,
                      args.rtscts, args.dsrdtr)

    print(f"Listening on {args.port} @ {args.baud} "
          f"({args.bytesize}{args.parity}{args.stopbits}, "
          f"rtscts={args.rtscts}, dsrdtr={args.dsrdtr}) ... (Ctrl+C to quit)")

    try:
        if args.quad:
            while True:
                data = ser.read(2)
                if len(data) != 2:
                    continue
                lo, hi = data[0], data[1]
                word = lo | (hi << 8)
                stc_TL = (word >> 0)  & 0x0F
                stc_TR = (word >> 4)  & 0x0F
                stc_BL = (word >> 8)  & 0x0F
                stc_BR = (word >> 12) & 0x0F

                decoded = []
                for label, nib in (('TL', stc_TL), ('TR', stc_TR),
                                   ('BL', stc_BL), ('BR', stc_BR)):
                    d, m, s = decode_stc(nib)
                    decoded.append(f"{label}=0x{nib:X} (bin {nib:04b}) -> dir:{d} mag:{m} speed:{s}")
                print(f"raw: 0x{lo:02X} 0x{hi:02X}  (word 0x{word:04X})    " + ' | '.join(decoded))
        else:
            while True:
                b = ser.read(1)
                if len(b) == 0:
                    continue
                val = b[0]
                dir_bit, mag, speed = decode_stc(val & 0x0F)
                print(f"raw: 0x{val:02X}  bin:{bits8(val)}   stc=0x{val & 0x0F:X} (bin {val & 0x0F:04b}) -> dir:{dir_bit} mag:{mag} speed:{speed}")
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == '__main__':
    main()
