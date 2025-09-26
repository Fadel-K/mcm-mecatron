# mcm-mecatron

Notes to use the python files:

sniffer:

# 8E1 (default here)
./stc_sniffer.py /dev/ttyUSB0 115200 --parity E --bytesize 8 --stopbits 1

# 7E1 (sometimes used with parity)
./stc_sniffer.py /dev/ttyUSB0 115200 --parity E --bytesize 7 --stopbits 1

# No parity (8N1)
./stc_sniffer.py /dev/ttyUSB0 115200 --parity N --bytesize 8 --stopbits 1

# Quad-word decoding (2 bytes => TL/TR/BL/BR nibbles)
./stc_sniffer.py /dev/ttyUSB0 115200 --parity E --quad


sender:
chmod +x stc_sender.py

# Sweep mag 0..5 at 5 Hz, toggle dir every wrap (8E1 default)
./stc_sender.py /dev/ttyUSB0 115200 --mode sweep

# Fixed: dir=1, mag=3 at 10 Hz with even parity
./stc_sender.py /dev/ttyUSB0 115200 --mode fixed --dir 1 --mag 3 --hz 10 --parity E

# Interactive (type commands: set dir 1 | set mag 4 | send | quad C,D,9,1 | mirror | burst 20 50 | show | quit)
./stc_sender.py /dev/ttyUSB0 115200 --mode interactive

# Quad mirror: send same nibble to TL/TR/BL/BR (2 bytes, low then high)
./stc_sender.py /dev/ttyUSB0 115200 --quad --mode fixed --dir 0 --mag 5

# Quad with explicit nibbles (hex 0..F) for TL,TR,BL,BR
./stc_sender.py /dev/ttyUSB0 115200 --quad --mode fixed --nibbles C,D,9,1
