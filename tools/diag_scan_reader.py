# tools/diag_scan_reader.py
import sys, serial, struct, time

def crc16_ccitt(data: bytes, poly=0x1021, init=0xFFFF):
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ poly) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

if len(sys.argv) < 2:
    print("Usage: python tools/diag_scan_reader.py COM23")
    sys.exit(1)

port = sys.argv[1]
ser = serial.Serial(port, 115200, timeout=1)
print("Listening on", port)

buf = bytearray()
while True:
    b = ser.read(1)
    if not b:
        continue
    buf += b
    # sync
    if len(buf) >= 2 and buf[-2:] == b'\\xA5\\x5A':
        # read header (6 bytes)
        hdr = ser.read(6)
        if len(hdr) < 6:
            buf = bytearray()
            continue
        version, ptype = hdr[0], hdr[1]
        seq = struct.unpack('<H', hdr[2:4])[0]
        plen = struct.unpack('<H', hdr[4:6])[0]
        payload = ser.read(plen)
        if len(payload) < plen:
            buf = bytearray()
            continue
        crc = ser.read(2)
        if len(crc) < 2:
            buf = bytearray()
            continue
        # optional: verify CRC
        hdr_payload = bytes([version, ptype]) + hdr[2:6] + payload
        if crc16_ccitt(hdr_payload) != struct.unpack('<H', crc)[0]:
            # CRC mismatch; skip
            continue
        if ptype == 0x06:
            if len(payload) >= 14:
                sc_ready = struct.unpack('<I', payload[0:4])[0]
                sc_call  = struct.unpack('<I', payload[4:8])[0]
                sc_sent  = struct.unpack('<I', payload[8:12])[0]
                last_pts = struct.unpack('<H', payload[12:14])[0]
                print(f"DIAG: scan_ready={sc_ready}  scan_call={sc_call}  scan_sent={sc_sent}  last_pts={last_pts}")
            else:
                print("DIAG: bad payload len", len(payload))
        # keep looping
        buf = bytearray()