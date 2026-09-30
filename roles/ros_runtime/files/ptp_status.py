#!/usr/bin/env python3
"""Publish a read-only, independently cross-checked PTP-to-UTC mapping."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time

NSEC = 1_000_000_000

def parse_pmc(text):
    fields = {}
    for key in ('currentUtcOffset', 'ptpTimescale', 'currentUtcOffsetValid', 'portState'):
        found = re.search(r'^\s*' + key + r'\s+(\S+)\s*$', text, re.MULTILINE)
        if found is None:
            raise ValueError('missing PTP field: ' + key)
        fields[key] = found.group(1)
    offset = int(fields['currentUtcOffset'])
    if not 1 <= offset <= 100:
        raise ValueError('PTP UTC offset is unavailable or implausible')
    return {'current_utc_offset': offset,
            'ptp_timescale': fields['ptpTimescale'] == '1',
            'utc_offset_valid': fields['currentUtcOffsetValid'] == '1',
            'port_state': fields['portState']}

def check_mapping(state, phc_delta_ns, tai_delta_ns, tolerance_ns):
    expected = state['current_utc_offset'] * NSEC
    if not state['ptp_timescale'] or state['port_state'] != 'MASTER':
        return False, 'PTP clock is not a PTP-timescale master'
    if abs(tai_delta_ns - expected) > tolerance_ns:
        return False, 'kernel TAI offset disagrees with ptp4l'
    if abs(phc_delta_ns - expected) > tolerance_ns:
        return False, 'PHC is not aligned with the UTC system clock'
    return True, 'PTP offset agrees with kernel TAI and measured PHC offset'

def relative_clock(clock_id):
    samples = []
    for _ in range(5):
        before = time.time_ns()
        stamp = time.clock_gettime_ns(clock_id)
        after = time.time_ns()
        samples.append((after - before, stamp - (before + after) // 2))
    return min(samples)

def sample(args):
    result = subprocess.run(['/usr/sbin/pmc', '-u', '-s', args.ptp_socket,
                             '-i', str(Path(args.output).parent / 'pmc-client.sock'),
                             '-t', '1', '-d', str(args.domain), '-b', '0',
                             'GET TIME_PROPERTIES_DATA_SET', 'GET PORT_DATA_SET'],
                            capture_output=True, text=True, timeout=2, check=True)
    state = parse_pmc(result.stdout)
    # Resolve the PHC from the configured NIC after every restart, not a fixed ptp index.
    devices = sorted(Path('/sys/class/net', args.interface, 'device/ptp').glob('ptp*'))
    if len(devices) != 1:
        raise RuntimeError('cannot resolve exactly one PHC for ' + args.interface)
    fd = os.open('/dev/' + devices[0].name, os.O_RDONLY)
    try:
        phc_bracket, phc_delta = relative_clock((~fd << 3) | 3)
    finally:
        os.close(fd)
    tai_bracket, tai_delta = relative_clock(time.CLOCK_TAI)
    valid, reason = check_mapping(state, phc_delta, tai_delta, args.tolerance_ns)
    if max(phc_bracket, tai_bracket) > args.tolerance_ns:
        valid, reason = False, 'clock-read uncertainty exceeds mapping tolerance'
    state.update(utc_offset_verified=valid, reason=reason,
                 phc_minus_utc_ns=phc_delta, kernel_tai_minus_utc_ns=tai_delta,
                 phc_read_bracket_ns=phc_bracket, tolerance_ns=args.tolerance_ns)
    return state

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--interface', required=True)
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--ptp-socket', default='/run/ptp4lro')
    parser.add_argument('--output', default='/run/uav/time/ptp-status.json')
    parser.add_argument('--tolerance-ns', type=int, default=1_000_000)
    args = parser.parse_args()
    output = Path(args.output)
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    last_reason = None
    while True:
        try:
            state = sample(args)
        except Exception as exc:
            state = {'utc_offset_verified': False, 'reason': str(exc)}
        state.update(schema_version=1, boot_id=boot_id,
                     updated_unix_ns=time.time_ns(), updated_monotonic_ns=time.monotonic_ns())
        temporary = output.with_suffix('.tmp')
        temporary.write_text(json.dumps(state) + '\n')
        os.chmod(temporary, 0o644)
        os.replace(temporary, output)
        if state['reason'] != last_reason:
            print(json.dumps(state), flush=True)
            last_reason = state['reason']
        time.sleep(0.5)

if __name__ == '__main__':
    main()
