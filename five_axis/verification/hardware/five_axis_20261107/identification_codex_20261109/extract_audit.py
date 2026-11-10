"""Archive only local audit evidence; never opens serial/ROS or sends commands."""
import gzip
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
DATA = HERE.parent / 'acquisition_20261109'


def main():
    manifest = json.loads((DATA / 'ACQUISITION_MANIFEST_20261109.json').read_text())
    files = {}
    for session in manifest['sessions']:
        client = json.loads((DATA / session['client']).read_text())
        source = client['backend_arm_receipt'].split('audit_file=')[1]
        files.setdefault(source, []).append(client['command_id'])
    index = []
    for source, commands in files.items():
        path = Path(source)
        destination = HERE / (path.stem + '_cycles.jsonl.gz')
        digest = hashlib.sha256()
        count = 0
        with path.open('rb') as inp, gzip.open(destination, 'wt', encoding='utf-8') as out:
            for line in inp:
                digest.update(line)
                row = json.loads(line)
                if row['kind'] == 'SESSION' or (row['kind'] == 'CONTROL_CYCLE' and row['command_id'] in commands):
                    out.write(json.dumps(row, separators=(',', ':')) + '\n')
                    count += 1
        index.append({'source': source, 'source_sha256': digest.hexdigest(),
                      'commands': commands, 'archive': destination.name,
                      'archive_sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
                      'records': count})
    (HERE / 'audit_sources.json').write_text(json.dumps(index, indent=2) + '\n')
    print(json.dumps(index, indent=2))


if __name__ == '__main__':
    main()
