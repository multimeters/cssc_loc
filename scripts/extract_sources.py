#!/usr/bin/env python3
"""Extract ROS package dependency closure from the pinned APS manifest.

Run after cloning the manifest repositories into --cache. Does not rewrite
upstream, remove existing files, or silently substitute ROS distro packages.
"""
import argparse
import hashlib
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
DEPEND_TAGS = {
    'depend', 'build_depend', 'buildtool_depend', 'build_export_depend',
    'buildtool_export_depend', 'exec_depend',
}


def git(directory, *args):
    return subprocess.check_output(['git', '-C', str(directory), *args], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--upstream', type=Path, default=ROOT.parent / 'upstream-autoware-aps')
    parser.add_argument('--cache', type=Path, default=ROOT.parent / 'upstream-repos')
    parser.add_argument('--profiles', type=Path, default=ROOT / 'config' / 'source_profiles.yaml')
    args = parser.parse_args()
    manifest = yaml.safe_load((args.upstream / 'autoware.repos').read_text())['repositories']
    profiles = yaml.safe_load(args.profiles.read_text())['profiles']
    repositories, packages = {}, {}
    for key, spec in manifest.items():
        directory = args.cache / Path(spec['url'].removesuffix('.git')).name
        if not (directory / '.git').exists():
            continue
        commit = git(directory, 'rev-parse', 'HEAD')
        expected = git(directory, 'rev-parse', spec['version'] + '^{commit}')
        if commit != expected:
            raise RuntimeError(f'{directory}: HEAD {commit} does not match {spec["version"]}')
        if git(directory, 'status', '--porcelain', '--untracked-files=no'):
            raise RuntimeError(f'{directory}: tracked source differs from the pinned commit')
        repositories[key] = dict(spec, commit=commit, directory=directory)
        tracked = git(directory, 'ls-files', '--', '**/package.xml', 'package.xml').splitlines()
        for relative_xml in sorted(tracked):
            xml = directory / relative_xml
            tree = ET.parse(xml).getroot()
            name = tree.findtext('name')
            if name in packages:
                raise RuntimeError(f'Duplicate package {name}')
            dependencies = sorted({x.text.strip() for x in tree if x.tag in DEPEND_TAGS})
            packages[name] = dict(repository=key, path=xml.parent, dependencies=dependencies)
    closures, external = {}, {}
    for profile, roots in profiles.items():
        missing = sorted(set(roots) - packages.keys())
        if missing:
            raise RuntimeError(f'Missing root packages for {profile}: {missing}')
        todo, selected, system = list(roots), set(), set()
        while todo:
            name = todo.pop()
            if name in selected:
                continue
            selected.add(name)
            for dep in packages[name]['dependencies']:
                if dep in packages:
                    todo.append(dep)
                else:
                    system.add(dep)
        # Missing Autoware source is not an ordinary system dependency.
        unresolved = [d for d in system if d.startswith(('autoware_', 'tier4_'))]
        if unresolved:
            raise RuntimeError(f'{profile}: unresolved Autoware dependencies: {unresolved}')
        closures[profile] = sorted(selected)
        external[profile] = sorted(system)
    selected = sorted(set().union(*map(set, closures.values())))
    used_repos, locked_packages = set(), {}
    for name in selected:
        package = packages[name]
        key = package['repository']
        repo = repositories[key]
        source = package['path']
        relative = source.relative_to(repo['directory'])
        destination = ROOT / 'src' / 'vendor' / name
        tracked_paths = git(repo['directory'], 'ls-files', '-z', '--', relative.as_posix()).split('\0')
        source_files = {Path(p).relative_to(relative) for p in tracked_paths if p}
        # Re-extraction is idempotent, but never overwrite local package edits.
        if destination.exists():
            existing_files = {p.relative_to(destination) for p in destination.rglob('*') if p.is_file()}
            if existing_files != source_files or any(
                (destination / p).read_bytes() != (source / p).read_bytes() for p in source_files
            ):
                raise RuntimeError(f'{destination} differs from upstream; preserve local edits first')
        else:
            for path in sorted(source_files):
                target = destination / path
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source / path, target)
        digest = hashlib.sha256()
        for path in sorted(destination.rglob('*')):
            if path.is_file():
                digest.update(path.relative_to(destination).as_posix().encode())
                digest.update(b'\0')
                digest.update(path.read_bytes())
        locked_packages[name] = {
            'repository': key, 'path': relative.as_posix(), 'commit': repo['commit'],
            'dependencies': package['dependencies'], 'content_sha256': digest.hexdigest(),
        }
        used_repos.add(key)
    locked_repos = {}
    for key in sorted(used_repos):
        repo = repositories[key]
        # Retain original upstream URLs and provide HTTPS for reproducible fetching.
        url = repo['url'].replace('git@github.com:', 'https://github.com/')
        locked_repos[key] = {'type': 'git', 'url': url, 'version': repo['commit']}
        licenses = ROOT / 'third_party' / Path(repo['url'].removesuffix('.git')).name
        licenses.mkdir(parents=True, exist_ok=True)
        for path in repo['directory'].iterdir():
            if path.is_file() and path.name.upper().startswith(('LICENSE', 'COPYING', 'NOTICE')):
                shutil.copy2(path, licenses / path.name)
    lock = {
        'upstream': {'url': 'https://github.com/libpet-co/autoware.APS.git',
                     'branch': 'hmi_container_dev', 'commit': git(args.upstream, 'rev-parse', 'HEAD')},
        'profiles': closures, 'external_dependencies': external, 'packages': locked_packages,
        'repositories': {key: {'url': repositories[key]['url'],
                               'requested_version': repositories[key]['version'],
                               'commit': repositories[key]['commit']} for key in sorted(used_repos)},
    }
    (ROOT / 'sources.lock.json').write_text(json.dumps(lock, indent=2) + '\n')
    (ROOT / 'localization.repos').write_text(yaml.safe_dump({'repositories': locked_repos}, sort_keys=False))
    (ROOT / 'config' / 'upstream.autoware.repos').write_bytes((args.upstream / 'autoware.repos').read_bytes())
    print(json.dumps({'packages': len(selected), 'repositories': len(used_repos),
                      'profiles': {p: len(c) for p, c in closures.items()}}, indent=2))


if __name__ == '__main__':
    main()
