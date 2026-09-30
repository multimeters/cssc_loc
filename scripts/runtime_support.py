"""Small shared helpers for immutable run settings and readable status files."""
import json
import os
from pathlib import Path
import shutil

import yaml


def save_configuration(config, output):
    output = Path(output)
    snapshot = {key: value for key, value in config.items() if not key.startswith('_')}
    snapshot['native_parameters'] = {}
    native = output / 'config' / 'native'
    native.mkdir(parents=True)
    for name, source in config['native_parameters'].items():
        target = native / (name + '.yaml')
        shutil.copy2(source, target)
        snapshot['native_parameters'][name] = str(target.resolve())
    destination = output / 'config' / 'localization.yaml'
    destination.write_text(yaml.safe_dump(snapshot, allow_unicode=True, sort_keys=False), encoding='utf8')
    return destination.resolve()


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf8')
    os.replace(temporary, path)
