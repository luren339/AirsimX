"""Keep one real GitHub star-count observation per UTC day. Never reads credentials or stargazer identities."""
import argparse
import csv
from datetime import date, datetime, timezone
from io import StringIO
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
MEDIA = ROOT / 'docs/airsimx/media'
CSV = MEDIA / 'stars.csv'
REPO = 'naipi11/AirsimX'


def load():
    if not CSV.exists():
        return {}
    with CSV.open(encoding='utf-8', newline='') as f:
        r = csv.DictReader(f)
        if r.fieldnames != ['date_utc', 'stars']:
            raise ValueError('unexpected CSV header')
        points = {}
        for row in r:
            d, v = date.fromisoformat(row['date_utc']), int(row['stars'])
            if d in points or v < 0:
                raise ValueError('duplicate date or negative star count')
            points[d] = v
    return points


def chart(points, dark):
    rows = sorted(points.items())
    start, end = rows[0][0], rows[-1][0]
    days = (end - start).days
    highest = max(1, max(points.values()))
    bg, fg, dim, rule, line = (('#1a1a19', '#f0efec', '#c3c2b7', '#484846', '#3987e5') if dark else
                                ('#fcfcfb', '#192f3c', '#445462', '#d4dce0', '#2a78d6'))
    left, right, top, bottom = 52, 576, 52, 125
    xy = [(right if not days else left + (right - left) * (d - start).days / days,
           bottom - (bottom - top) * v / highest) for d, v in rows]
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="620" height="166" viewBox="0 0 620 166" role="img" aria-labelledby="t desc"><title id="t">GitHub star observations for {REPO}</title>',
             f'<desc id="desc">{len(rows)} observed UTC dates. Latest: {end}, {rows[-1][1]} stars. No history before the first observation.</desc>',
             f'<rect width="620" height="166" rx="8" fill="{bg}"/><g font-family="system-ui, sans-serif" fill="{fg}">',
             '<text x="24" y="28" font-size="17" font-weight="650">GitHub stars</text>',
             f'<text x="595" y="28" text-anchor="end" font-size="15">{rows[-1][1]} as of {end}</text>',
             f'<path d="M{left} {bottom}H{right}" fill="none" stroke="{rule}"/><text x="36" y="130" text-anchor="end" font-size="13" fill="{dim}">0</text>']
    if len(rows) == 1:
        parts.append(f'<text x="52" y="85" font-size="16" fill="{dim}">First observation; no trend yet.</text>')
    else:
        pairs = ' '.join(f'{x:.2f},{y:.2f}' for x, y in xy)
        parts.extend([f'<path d="M{left} {top}H{right}" stroke="{rule}"/>',
                      f'<text x="36" y="57" text-anchor="end" font-size="13" fill="{dim}">{highest}</text>',
                      f'<polyline points="{pairs}" fill="none" stroke="{line}" stroke-width="2"/>'])
    for (d, n), (x, y) in zip(rows, xy):
        parts.append(f'<g><title>{d} UTC: {n} stars</title><circle cx="{x:.2f}" cy="{y:.2f}" r="5" fill="{line}" stroke="{bg}" stroke-width="2"/><circle cx="{x:.2f}" cy="{y:.2f}" r="10" fill="transparent"/></g>')
    if days:
        parts.append(f'<text x="{left}" y="152" fill="{dim}" font-size="14">{start}</text>')
    parts.append(f'<text x="{right}" y="152" text-anchor="end" fill="{dim}" font-size="14">{end}</text></g></svg>')
    return '\n'.join(parts) + '\n'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--observe', action='store_true', help='read the current public count using GitHub CLI')
    a = p.parse_args()
    points = load()
    if a.observe:
        value = json.loads(subprocess.check_output(['gh', 'api', f'repos/{REPO}', '--jq',
            '{repo:.full_name,stars:.stargazers_count}'], text=True, timeout=30))
        if value['repo'] != REPO or type(value['stars']) is not int or value['stars'] < 0:
            raise ValueError('unexpected GitHub repository or star count')
        points[datetime.now(timezone.utc).date()] = value['stars']
        s = StringIO(newline=''); w = csv.writer(s, lineterminator='\n')
        w.writerow(('date_utc', 'stars')); w.writerows(sorted(points.items()))
        CSV.write_text(s.getvalue(), encoding='utf-8', newline='\n')
    if not points:
        p.error('no observations yet; run with --observe while GitHub CLI is available')
    for dark in False, True:
        name = 'star-history-dark.svg' if dark else 'star-history.svg'
        (MEDIA / name).write_text(chart(points, dark), encoding='utf-8', newline='\n')
    print(f'{REPO}: {max(points)} UTC, {points[max(points)]} stars; {len(points)} observed dates')


if __name__ == '__main__':
    main()
