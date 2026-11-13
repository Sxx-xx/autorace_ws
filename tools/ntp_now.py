#!/usr/bin/python3
"""ntp_now.py : print the true time now (seconds since 1970), for setting the PC clock.

The PC's clock drifted seconds off on the phone hotspots (2.8 s, 5.1 s, 2026-11-10/13)
while saying it was synchronised, and the robot's messages then looked seconds old.
This asks a time server directly (SNTP, corrected for half the round trip); if no
server answers, it asks the robot's clock over ssh, corrected the same way.

  sudo date -s "@$(python3 tools/ntp_now.py)"        set the PC clock
  python3 tools/ntp_now.py --check                   only print how far off it is
"""
import socket
import struct
import subprocess
import sys
import time

SERVERS = ('time.google.com', 'pool.ntp.org', 'time.windows.com')
ROBOT = 'dd@turtlebot1.local'


def from_server(host):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(2.0)
    t1 = time.time()
    s.sendto(b'\x1b' + 47 * b'\0', (host, 123))
    data, _ = s.recvfrom(48)
    t4 = time.time()
    sec, frac = struct.unpack('!II', data[40:48])
    server = sec - 2208988800 + frac / 2 ** 32
    return server - (t1 + t4) / 2.0, t4 - t1         # offset, round trip


def from_robot():
    """The robot's clock over one open ssh connection: a line sent, its time
    back; the quickest of several exchanges, corrected for half its round trip."""
    proc = subprocess.Popen(
        ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', ROBOT,
         'python3 -u -c "import sys,time\nprint(0)\nfor l in sys.stdin: print(repr(time.time()))"'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    try:
        if proc.stdout.readline().strip() != '0':
            raise RuntimeError('no answer from the robot')
        best = None
        for _ in range(10):
            t1 = time.time()
            proc.stdin.write('x\n')
            proc.stdin.flush()
            robot = float(proc.stdout.readline())
            t4 = time.time()
            if best is None or t4 - t1 < best[1]:
                best = (robot - (t1 + t4) / 2.0, t4 - t1)
        return best
    finally:
        proc.kill()


def main():
    best = None
    for host in SERVERS:
        for _ in range(3):
            try:
                offset, rtt = from_server(host)
            except OSError:
                continue
            if best is None or rtt < best[1]:
                best = (offset, rtt, host)
    if best is None or best[1] > 0.5:
        try:
            offset, rtt = from_robot()
            if best is None or rtt < best[1]:
                best = (offset, rtt, 'robot clock (ssh)')
        except Exception as error:          # noqa: BLE001 - report and go on
            print(f'robot: {error}', file=sys.stderr)
    if best is None:
        print('no time source answered', file=sys.stderr)
        sys.exit(1)
    offset, rtt, source = best
    print(f'{source}: the PC is {-offset:+.3f} s off (round trip {rtt:.3f} s)', file=sys.stderr)
    if '--check' not in sys.argv:
        print(f'{time.time() + offset:.3f}')


if __name__ == '__main__':
    main()
