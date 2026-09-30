#!/usr/bin/env python3
"""Dispatch live subscription or recorded-data validation from the same YAML."""
import argparse
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'aps_bag_localization'))
from aps_bag_localization.configuration import load_config


def main():
    selector = argparse.ArgumentParser(add_help=False)
    selector.add_argument('--mode', choices=('live', 'replay'))
    selector.add_argument('--config', type=Path, default=ROOT / 'config/localization.yaml')
    selected, _ = selector.parse_known_args()
    try:
        config = load_config(selected.config, mode=selected.mode)
    except (ValueError, KeyError, OSError, TypeError, yaml.YAMLError) as error:
        selector.error('配置读取失败：' + str(error))
    # Strip only --mode; keep --config and all mode-specific arguments for the
    # final parser, which rejects inappropriate live/replay options explicitly.
    mode_parser = argparse.ArgumentParser(add_help=False)
    mode_parser.add_argument('--mode', choices=('live', 'replay'))
    _, remaining = mode_parser.parse_known_args()
    sys.argv[1:] = remaining
    if config['runtime']['mode'] == 'live':
        from run_live_localization import main as run
    else:
        from run_fusion_replay import main as run
    return run()


if __name__ == '__main__':
    raise SystemExit(main())
