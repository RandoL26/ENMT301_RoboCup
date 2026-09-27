# save as read_diag.py
import serial, struct, time, argparse, sys

PORT = 'COM23'        # default port (change or pass --port)
BAUD = 115200

# CRC16-CCITT (poly 0x1021) initial 0xFFFF
def crc16_ccitt(data: bytes):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def read_frame(ser):
    # find sync 0xA5 0x5A
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == 0xA5:
            b2 = ser.read(1)
            if b2 and b2[0] == 0x5A:
                break
    hdr = ser.read(6)   # version(1), type(1), seq(2), payload_len(2)
    if len(hdr) != 6:
        return None
    # header layout: u8 version, u8 packet_type, u16 seq, u16 payload_len
    version, ptype, seq, payload_len = struct.unpack('<BBHH', hdr)
    payload = ser.read(payload_len)
    crc_bytes = ser.read(2)
    if len(payload) != payload_len or len(crc_bytes) != 2:
        return None
    # verify CRC over header(6)+payload
    # reconstruct header bytes for CRC verification
    hdr_payload = hdr + payload
    if crc16_ccitt(hdr_payload) != struct.unpack('<H', crc_bytes)[0]:
        # bad crc
        return None
    return ptype, payload

def parse_diag(payload):
    # expected layout (62 bytes minimum):
    # u32 scan_ready, u32 call_count, u32 sent_count, u16 last_pts,
    # followed by 13 u16 fields (40 bytes) => angle_diff_std
    # then 11 u16 fields for scan-order diagnostics:
    # num_points, first_angle_cdeg, last_angle_cdeg, min_angle_cdeg, max_angle_cdeg,
    # neg_steps_count, large_pos_jumps_count, largest_pos_step_cdeg, largest_neg_step_cdeg,
    # total_span_cdeg, approx_one_revolution
    if len(payload) < 62:
        return None
    header_fmt = '<IIIH13H11H'
    unpacked = struct.unpack(header_fmt, payload[:62])
    scan_ready = unpacked[0]
    call_count = unpacked[1]
    sent_count = unpacked[2]
    last_pts = unpacked[3]
    (zero_count, invalid_count, min_mm, max_mm, large_jump,
     matched_count, max_delta_mm, cnt_gt_100, cnt_gt_500, cnt_gt_1000,
     angle_of_max_delta_cdeg, angle_diff_mean_cdeg, angle_diff_std_cdeg) = unpacked[4:17]
    (num_points, first_angle_cdeg, last_angle_cdeg, min_angle_cdeg, max_angle_cdeg,
     neg_steps_count, large_pos_jumps_count, largest_pos_step_cdeg, largest_neg_step_cdeg,
     total_span_cdeg, approx_one_revolution) = unpacked[17:28]

    return {
        'scan_ready_count': scan_ready,
        'scan_telemetry_call_count': call_count,
        'scan_telemetry_sent_count': sent_count,
        'last_scan_point_count': last_pts,
        'ld_zero_count': zero_count,
        'ld_invalid_count': invalid_count,
        'ld_min_mm': min_mm,
        'ld_max_mm': max_mm,
        'ld_large_jump_count': large_jump,
        'ld_matched_count': matched_count,
        'ld_max_delta_mm': max_delta_mm,
        'ld_count_delta_gt_100': cnt_gt_100,
        'ld_count_delta_gt_500': cnt_gt_500,
        'ld_count_delta_gt_1000': cnt_gt_1000,
        'ld_angle_of_max_delta_cdeg': angle_of_max_delta_cdeg,
        'ld_angle_diff_mean_cdeg': angle_diff_mean_cdeg,
        'ld_angle_diff_std_cdeg': angle_diff_std_cdeg,
        # scan-order diagnostics
        'ld06_num_points': num_points,
        'ld06_first_angle_cdeg': first_angle_cdeg,
        'ld06_last_angle_cdeg': last_angle_cdeg,
        'ld06_min_angle_cdeg': min_angle_cdeg,
        'ld06_max_angle_cdeg': max_angle_cdeg,
        'ld06_neg_steps_count': neg_steps_count,
        'ld06_large_pos_jumps_count': large_pos_jumps_count,
        'ld06_largest_pos_step_cdeg': largest_pos_step_cdeg,
        'ld06_largest_neg_step_cdeg': largest_neg_step_cdeg,
        'ld06_total_span_cdeg': total_span_cdeg,
        'ld06_approx_one_revolution': approx_one_revolution,
    }

def main():
    ap = argparse.ArgumentParser(description='Read LD06 diagnostic frames and optionally write CSV')
    ap.add_argument('--port', '-p', default=None, help='Serial port (e.g. COM3)')
    ap.add_argument('--baud', '-b', type=int, default=BAUD, help='Baud rate')
    ap.add_argument('--out', '-o', default=None, help='Optional output file path (CSV). If omitted, prints to stdout')
    args = ap.parse_args()

    port = args.port or PORT
    baud = args.baud
    out_path = args.out
    out_fh = None
    if out_path:
        try:
            out_fh = open(out_path, 'w', buffering=1)
            # header
            out_fh.write('timestamp,scan_ready,call_count,sent_count,last_pts,zero,invalid,min_mm,max_mm,large_jump,matched_count,max_delta_mm,cnt_gt_100,cnt_gt_500,cnt_gt_1000,angle_of_max_cdeg,angle_mean_cdeg,angle_std_cdeg\n')
        except Exception as e:
            print(f'Failed to open output file {out_path}: {e}', file=sys.stderr)
            out_fh = None

    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            print('Listening on', port)
            while True:
                r = read_frame(ser)
                if not r:
                    continue
                ptype, payload = r
                if ptype == 0x06:
                    d = parse_diag(payload)
                    if d:
                        now = time.strftime('%Y-%m-%d %H:%M:%S')
                        if out_fh:
                            out_fh.write(','.join([
                                now,
                                str(d['scan_ready_count']),
                                str(d['scan_telemetry_call_count']),
                                str(d['scan_telemetry_sent_count']),
                                str(d['last_scan_point_count']),
                                str(d['ld_zero_count']),
                                str(d['ld_invalid_count']),
                                str(d['ld_min_mm']),
                                str(d['ld_max_mm']),
                                str(d['ld_large_jump_count']),
                                str(d['ld_matched_count']),
                                str(d['ld_max_delta_mm']),
                                str(d['ld_count_delta_gt_100']),
                                str(d['ld_count_delta_gt_500']),
                                str(d['ld_count_delta_gt_1000']),
                                str(d['ld_angle_of_max_delta_cdeg']),
                                str(d['ld_angle_diff_mean_cdeg']),
                                str(d['ld_angle_diff_std_cdeg'])
                            ]) + '\n')
                        else:
                            print(now, d)
    except serial.SerialException as e:
        print(f'Serial error: {e}', file=sys.stderr)


if __name__ == '__main__':
    main()