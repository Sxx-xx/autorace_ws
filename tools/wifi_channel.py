#!/usr/bin/env python3
"""wifi_channel.py : pick the robot's Wi-Fi channel from a scan of every channel.

Order:  1. a quiet non-DFS 5 GHz channel        (36-48, 149-165)
        2. a quiet DFS channel                  (52-64, 100-144; weather radar 120-128 last)
        3. the least busy non-DFS channel
        4. the least busy DFS channel
        5. 2.4 GHz 1 / 6 / 11                   (only when the robot can use no 5 GHz channel)
DFS comes after non-DFS: when the access point hears a radar it must leave the channel and listen on
the new one for a minute (ten on 120-128) before it sends again, and a client may not call first on a
DFS channel, it has to hear the access point. Channels the robot's Wi-Fi cannot use (`iw list`) are
left out.

Scan where the robot runs, with the robot's own Wi-Fi (default; asks the Pi's sudo password once):
  python3 tools/wifi_channel.py                         # ROBOT=dd@turtlebot1.local
  python3 tools/wifi_channel.py --apply robot-ap        # and set the laptop AP (Navigator) to it
  python3 tools/wifi_channel.py --width 40              # for a 40 MHz access point
  python3 tools/wifi_channel.py --scan pc               # scan with this PC (nmcli) instead
  python3 tools/wifi_channel.py --file scan.txt         # a saved `iw dev wlan0 scan` / nmcli -t list
The robot's own network (the SSID it is on, and --own NAME) is not counted as crowding.
The ipTIME router is set by hand; the phone hotspot cannot be given a channel (it prints how/why).
"""
import argparse
import os
import re
import shutil
import subprocess
import sys

NON_DFS = [36, 40, 44, 48, 149, 153, 157, 161, 165]
DFS = [52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144]
WEATHER = {120, 124, 128}
G24 = [1, 6, 11]
PAIR40 = [(36, 40), (44, 48), (52, 56), (60, 64), (100, 104), (108, 112), (116, 120),
          (124, 128), (132, 136), (140, 144), (149, 153), (157, 161)]
TIER = ['quiet non-DFS', 'quiet DFS', 'quiet DFS (weather radar)', 'busy non-DFS', 'busy DFS',
        '2.4 GHz']


def channel(freq):
    f = int(float(freq))
    if 2412 <= f <= 2472:
        return (f - 2407) // 5
    if f == 2484:
        return 14
    if 5150 <= f <= 5900:
        return (f - 5000) // 5
    return None


def parse_iw_scan(text):
    """`iw dev wlan0 scan` (several scans one after another are fine: the strongest is kept)."""
    aps, ap = {}, None
    for line in text.splitlines():
        m = re.match(r'BSS ([0-9a-fA-F:]{17})', line)
        if m:
            ap = aps.setdefault(m.group(1).lower(), {})
            continue
        if ap is None:
            continue
        s = line.strip().lstrip('* ').strip()
        val = s.split(':', 1)[1].strip() if ':' in s else ''
        if s.startswith('freq:'):
            ap['ch'] = channel(val.split()[0])
        elif s.startswith('signal:'):
            ap['signal'] = max(float(val.split()[0]), ap.get('signal', -200.0))
        elif s.startswith('SSID:'):
            ap['ssid'] = val
        elif s.startswith('secondary channel offset:'):
            ap['offset'] = val.split()[0]
        elif s.startswith('channel width:') and '(' in val:
            ap['vht_width'] = int(val.split()[0])
        elif s.startswith('center freq segment 1:'):
            ap['seg1'] = int(val.split()[0])
        elif s.startswith('center freq segment 2:'):
            ap['seg2'] = int(val.split()[0])
        elif s.startswith('channel utilisation:'):
            ap['util'] = int(val.split('/')[0])
    return [a for a in aps.values() if a.get('ch')]


def parse_nmcli(text):
    """`nmcli -t -f CHAN,SIGNAL,SSID dev wifi list` (signal in %, no channel width)."""
    aps = []
    for line in text.splitlines():
        parts = line.split(':', 2)
        if len(parts) == 3 and parts[0].isdigit() and parts[1].isdigit():
            aps.append({'ch': int(parts[0]), 'signal': int(parts[1]) / 2 - 100,
                        'ssid': parts[2].replace('\\:', ':')})
    return aps


def parse_caps(text):
    """`iw reg get` + `iw list`: country and, per channel, ok / radar / disabled."""
    country = re.search(r'country (\w\w)', text)
    caps = {}
    for m in re.finditer(r'\*\s+(\d+)(?:\.\d+)? MHz \[(\d+)\](.*)', text):
        rest, ch = m.group(3), int(m.group(2))
        state = 'disabled' if 'disabled' in rest else ('radar' if 'radar' in rest else 'ok')
        if caps.get(ch) != 'ok':          # several radios: usable if any can
            caps[ch] = state
    return (country.group(1) if country else None), caps


def occupied(ap):
    """The 20 MHz channels an access point sends on (2.4 GHz: every channel number it touches)."""
    p = ap['ch']
    if p <= 14:
        lo, hi = p - 2, p + 2
        if ap.get('offset') == 'above':
            hi = p + 6
        elif ap.get('offset') == 'below':
            lo = p - 6
        return set(range(max(1, lo), hi + 1))
    w, s1, s2 = ap.get('vht_width', 0), ap.get('seg1', 0), ap.get('seg2', 0)
    if w >= 1 and s1:
        if s2 and abs(s2 - s1) == 8:
            c, half = s2, 14              # 160 MHz, new style: segment 2 is its centre
        elif w == 2:
            c, half = s1, 14              # 160 MHz, old style
        else:
            c, half = s1, 6               # 80 MHz
        return set(range(c - half, c + half + 1, 4))
    if ap.get('offset') == 'above':
        return {p, p + 4}
    if ap.get('offset') == 'below':
        return {p - 4, p}
    return {p}


def weight(ap):
    """How much one access point crowds a channel: 0 at -95 dBm, 1 at -70, 2 at -45 and over;
    more when it reports its channel busy (BSS load)."""
    w = min(2.0, max(0.0, (ap.get('signal', -95.0) + 95.0) / 25.0))
    if ap.get('util') is not None:
        w *= 1.0 + ap['util'] / 255.0
    return w


def candidates(width, caps):
    out = []
    groups = [(c,) for c in NON_DFS + DFS] if width == 20 else PAIR40
    for chs in groups:
        if caps and any(caps.get(c, 'disabled') == 'disabled' for c in chs):
            continue
        out.append(chs)
    for c in G24:
        if not caps or caps.get(c, 'disabled') != 'disabled':
            out.append((c,))
    return out


def rank(aps, width, caps, quiet):
    rows = []
    for chs in candidates(width, caps):
        span = set(range(chs[0] - 2, chs[0] + 3)) if chs[0] <= 14 else set(chs)
        near = [a for a in aps if occupied(a) & span]
        score = sum(weight(a) for a in near)
        dfs = any(c in DFS for c in chs)
        weather = any(c in WEATHER for c in chs)
        if chs[0] <= 14:
            tier = 5
        elif score <= quiet:
            tier = 2 if weather else (1 if dfs else 0)
        else:
            tier = 4 if dfs else 3
        rows.append({'chs': chs, 'score': score, 'n': len(near), 'tier': tier, 'dfs': dfs,
                     'weather': weather,
                     'strongest': max((a['signal'] for a in near), default=None)})
    rows.sort(key=lambda r: (r['tier'], r['weather'], r['score'], r['chs'][0]))
    return rows


def run(cmd, **kw):
    return subprocess.run(cmd, text=True, capture_output=True, **kw)


SCAN = ('for n in 1 2; do for t in 1 2 3; do {sudo} iw dev wlan0 scan && break; sleep 2; done; '
        'sleep 1; done')


def from_robot(robot):
    ssh = ['ssh', '-o', 'ConnectTimeout=8']
    info = run(ssh + ['-o', 'BatchMode=yes', robot,
                      'iw reg get; iw list; echo ===LINK; iw dev wlan0 link'])
    if info.returncode != 0:
        sys.exit(f'ssh to {robot} failed: {info.stderr.strip()} (ROBOT=user@host)')
    caps_text, _, link = info.stdout.partition('===LINK')
    ssid = re.search(r'SSID: (.*)', link)
    scan = run(ssh + ['-o', 'BatchMode=yes', robot, SCAN.format(sudo='sudo -n')]).stdout
    if 'BSS ' not in scan:
        print(f'scanning on {robot}: the Pi asks for its sudo password', flush=True)
        subprocess.run(ssh + ['-t', robot,
                              f'( {SCAN.format(sudo="sudo")} ) > /tmp/wifi_scan.txt'])
        scan = run(ssh + [robot, 'cat /tmp/wifi_scan.txt; rm -f /tmp/wifi_scan.txt']).stdout
    if 'BSS ' not in scan:
        sys.exit('no scan from the robot (iw dev wlan0 scan failed)')
    return parse_iw_scan(scan), caps_text, (ssid.group(1).strip() if ssid else None)


def from_pc():
    if shutil.which('nmcli'):
        out = run(['nmcli', '-t', '-f', 'CHAN,SIGNAL,SSID', 'dev', 'wifi', 'list',
                   '--rescan', 'yes']).stdout
        print('scanned with this PC (nmcli: no channel widths, 80 MHz neighbours count on one '
              'channel only; a scan by the robot is better)')
        return parse_nmcli(out)
    sys.exit('no nmcli here; use --scan robot or --file')


def apply_ap(profile, ranked):
    for r in [r for r in ranked if r['tier'] < 5][:4] or ranked[:1]:
        c = r['chs'][0]
        band = 'bg' if c <= 14 else 'a'
        if r['dfs']:
            print(f'  channel {c} is DFS: many laptop Wi-Fi cards will not run an AP there')
        mod = run(['nmcli', 'connection', 'modify', profile, '802-11-wireless.band', band,
                   '802-11-wireless.channel', str(c)])
        if mod.returncode != 0:
            print(f'  nmcli modify {profile} failed: {mod.stderr.strip()}')
            return False
        up = run(['nmcli', 'connection', 'up', profile])
        if up.returncode == 0:
            print(f'  {profile} is up on channel {c}')
            return True
        print(f'  {profile} did not come up on channel {c} ({up.stderr.strip()[:80]}); next one')
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--scan', choices=['robot', 'pc'], default='robot')
    ap.add_argument('--robot', default=os.environ.get('ROBOT', 'dd@turtlebot1.local'))
    ap.add_argument('--file', help='a saved iw scan or nmcli -t list instead of scanning')
    ap.add_argument('--caps', help='a saved `iw reg get; iw list` of the robot (with --file)')
    ap.add_argument('--width', type=int, choices=[20, 40], default=20,
                    help='the access point channel width, MHz (default 20: steadier in a crowd)')
    ap.add_argument('--quiet', type=float, default=1.0,
                    help='at most this crowding counts as quiet (1.0 = one network at -70 dBm)')
    ap.add_argument('--own', action='append', default=[], help='our own SSID, not counted')
    ap.add_argument('--apply', metavar='PROFILE', help='set this NetworkManager AP profile '
                    '(robot-ap = Navigator) to the chosen channel, the next one if it fails')
    a = ap.parse_args()

    caps_text, on = '', None
    if a.file:
        text = open(a.file).read()
        aps = parse_iw_scan(text) if 'BSS ' in text else parse_nmcli(text)
        caps_text = open(a.caps).read() if a.caps else ''
    elif a.scan == 'robot':
        aps, caps_text, on = from_robot(a.robot)
    else:
        aps = from_pc()
        caps_text = run(['ssh', '-o', 'ConnectTimeout=8', '-o', 'BatchMode=yes', a.robot,
                         'iw reg get; iw list']).stdout
    country, caps = parse_caps(caps_text)
    own = set(a.own) | ({on} if on else set())
    aps = [x for x in aps if x.get('ssid') not in own]

    print(f'{len(aps)} other networks heard' + (f' (not counting {", ".join(sorted(own))})'
                                                 if own else ''))
    if not caps:
        print("the robot's usable channels are unknown (no iw list): every channel is offered")
    elif country != 'KR':
        print(f"robot Wi-Fi country {country or '?'}, not KR: some 5 GHz channels may be closed "
              'to it (docs/INTEGRATION_2.md, regulatory-domain: "KR")')
    ranked = rank(aps, a.width, caps, a.quiet)

    print(f'\n{"channel":>9} {"kind":<12} {"robot":<7} {"nets":>4} {"strongest":>9} '
          f'{"crowding":>8}  class')
    for r in sorted(ranked, key=lambda r: r['chs']):
        chs = '+'.join(map(str, r['chs']))
        kind = '2.4 GHz' if r['chs'][0] <= 14 else (
            'DFS-weather' if r['weather'] else ('DFS' if r['dfs'] else 'non-DFS'))
        robot = '?' if not caps else ('radar' if any(caps.get(c) == 'radar' for c in r['chs'])
                                      else 'ok')
        st = f"{r['strongest']:.0f} dBm" if r['strongest'] is not None else '-'
        print(f'{chs:>9} {kind:<12} {robot:<7} {r["n"]:>4} {st:>9} {r["score"]:>8.2f}  '
              f'{TIER[r["tier"]]}')

    if not ranked:
        sys.exit('no channel the robot can use')
    best = ranked[0]
    c = best['chs'][0]
    print(f'\nCHOSEN: channel {c}' + (f' ({a.width} MHz: {best["chs"][0]}+{best["chs"][1]})'
                                       if len(best['chs']) > 1 else '') +
          f'  [{TIER[best["tier"]]}, crowding {best["score"]:.2f}]')
    print('next:   ' + ', '.join(f"{r['chs'][0]} ({TIER[r['tier']]})" for r in ranked[1:4]))
    if best['dfs']:
        print('        DFS: the access point may move away on a radar; check the robot joins '
              'it before a run')
    print(f'\n  ipTIME: admin page (192.168.0.1) -> 5 GHz wireless settings: channel {c}, '
          f'width {a.width} MHz (fixed, not auto)')
    print('  iPhone hotspot: no channel setting; Maximize Compatibility off = 5 GHz. If its '
          'channel is busy, use the ipTIME or the laptop AP')
    if a.apply:
        apply_ap(a.apply, ranked)
    else:
        print(f'  laptop AP: python3 tools/wifi_channel.py --apply robot-ap  (or nmcli con '
              f'modify robot-ap 802-11-wireless.band a 802-11-wireless.channel {c})')


if __name__ == '__main__':
    main()
