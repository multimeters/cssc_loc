#!/usr/bin/env python3
"""Verify extracted vendor package bytes against sources.lock.json."""
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    lock = json.loads((root / 'sources.lock.json').read_text())
    mismatches = []
    for name, spec in sorted(lock['packages'].items()):
        package = root / 'src' / 'vendor' / name
        digest = hashlib.sha256()
        for path in sorted(package.rglob('*')):
            if path.is_file():
                digest.update(path.relative_to(package).as_posix().encode())
                digest.update(b'\0')
                digest.update(path.read_bytes())
        if digest.hexdigest() != spec['content_sha256']:
            mismatches.append(name)
    print(json.dumps({'packages_checked': len(lock['packages']), 'mismatches': mismatches}, indent=2))
    return int(bool(mismatches))


if __name__ == '__main__':
    raise SystemExit(main())
