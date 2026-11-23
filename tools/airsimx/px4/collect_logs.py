"""Copy PX4 console and flight logs from a run directory into a local evidence folder."""

import argparse
import shutil
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs-root', type=Path, default=Path.home() / 'airsimx/px4-root/runs')
    parser.add_argument('--run', required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    source = args.runs_root / args.run
    if not source.is_dir():
        raise FileNotFoundError(source)
    args.destination.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source / 'console.log', args.destination / (args.run + '-console.log'))
    for log in sorted((source / 'log').rglob('*.ulg')):
        shutil.copyfile(log, args.destination / (args.run + '-' + log.name))
    print(sorted(path.name for path in args.destination.iterdir()))


if __name__ == '__main__':
    main()
