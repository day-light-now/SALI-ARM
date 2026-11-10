"""Offline source/model fingerprints and byte-for-byte delivery comparisons."""
import argparse
import hashlib
import importlib.metadata as metadata
import json
import platform
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta, timezone
from .core import ROOT

DESKTOP=Path('C:/Users/ASUS/Desktop/SLAI机械臂控制/03_桌面上位机')
DESIGN=Path('C:/Users/ASUS/Documents/炒股选股/SLAI_UI_Design')


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def git(*args):
    return subprocess.check_output(['git','-c','core.quotepath=false',*args],cwd=ROOT,
                                   encoding='utf-8',errors='replace').strip()


def now():
    return datetime.now(timezone(timedelta(hours=8))).isoformat()


def write_json(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def runtime_files():
    files=[ROOT/'slai_console'/name for name in ('__init__.py','app.py','core.py','robot_contract.py','adapters.py','launch.py','capture_offline.py',
                                              'commissioning.py','shadow_worker.py','clock.py')]
    files += [ROOT/p for p in ('verification/impedance_demo/model.py',
              'verification/gravity_preparation/provisional_gravity.json',
              'verification/hardware/four_axis_monitor.py','verification/hardware/feedback_decode.py',
              'src/slaihand_bringup/config/hardware_mapping.yaml',
              'src/slaihand_description/urdf/slaihand_four_axis.urdf')]
    urdf=ET.parse(files[-1]).getroot()
    files += [ROOT/'src/slaihand_description/meshes'/Path(m.attrib['filename']).name
              for m in urdf.findall('.//visual/geometry/mesh')]
    return sorted(set(files))


def runtime_manifest():
    from .adapters import mujoco
    import PySide6
    from PySide6.QtCore import qVersion
    versions={}
    for name in ('PySide6','PySide6_Essentials','PySide6_Addons','shiboken6','numpy','scipy',
                 'pyserial','mujoco','glfw','PyOpenGL','etils','absl-py','Pillow'):
        try:
            versions[name]=metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name]=None
    return dict(python=sys.version,executable=sys.executable,platform=platform.platform(),pyside6_runtime=PySide6.__version__,
                qt_runtime=qVersion(),mujoco_runtime=mujoco.__version__,mujoco_path=mujoco.__file__,packages=versions)


def fingerprint(environment=None):
    files={str(p.relative_to(ROOT)).replace('\\','/'):sha(p) for p in runtime_files()}
    environment=environment or runtime_manifest()
    inputs=dict(files=files,python=environment['python'],platform=environment['platform'],
                qt=environment['qt_runtime'],packages=environment['packages'])
    digest=hashlib.sha256(json.dumps(inputs,sort_keys=True,ensure_ascii=False).encode()).hexdigest()
    return dict(sha256=digest,inputs=inputs)


def inspect_delivery(output):
    output=Path(output)
    output.mkdir(parents=True,exist_ok=True)
    pairs=[]
    def compare(kind,a,b):
        pairs.append(dict(kind=kind,delivery=str(a),source=str(b),
            delivery_sha256=sha(a) if a.is_file() else None,source_sha256=sha(b) if b.is_file() else None,
            equal=a.is_file() and b.is_file() and sha(a)==sha(b)))
    for d,s in [('完整使用说明.md','slai_console/README.md'),('复核修正记录.md','slai_console/REVIEW_RESOLUTION.md'),
                ('交付记录_2026-10-30.md','slai_console/交付记录_2026-10-30.md')]:
        compare('documentation',DESKTOP/d,ROOT/s)
    for name in ('启动桌面上位机.bat','启动桌面上位机.vbs','先读我.md'):
        compare('launcher',DESKTOP/name,ROOT/'desktop_entries/03_桌面上位机'/name)
    for name in ('实现复核_20261030.md','修正复核_20261030.md','SLAI_上位机_界面设计方案.md'):
        compare('reference',DESKTOP/'设计参考'/name,DESIGN/name)
    for d,s in [('DeepSeek首轮复核.md','实现复核_20261030.md'),('DeepSeek修正复核.md','修正复核_20261030.md')]:
        compare('review',DESKTOP/d,DESIGN/s)
    for i in range(1,7):
        compare('screenshot_copy',DESKTOP/'验证截图'/f'{i:02d}.png',ROOT/'tmp/desktop_console/screenshots'/f'{i:02d}.png')
    for p in sorted((DESIGN/'mockups').glob('*.png')):
        compare('design_mockup',DESKTOP/'设计参考/mockups'/p.name,p)
    result=dict(at=now(),base_commit=git('rev-parse','HEAD'),
        working_tree=git('status','--porcelain=v1'),comparisons=pairs,
        original_source_copy_present=(DESKTOP/'源码快照').exists() or any(DESKTOP.glob('版本快照/*/源码快照')),
        old_screenshot_source_provenance='unbound: no runtime source/environment fingerprint was recorded',
        current_runtime=fingerprint())
    destination=output/'initial_comparison.json'
    if destination.exists():
        raise FileExistsError('Preserve initial evidence; choose a new output directory')
    write_json(destination,result)
    archive=output/'original_delivery'
    archive.mkdir()
    for p in DESKTOP.glob('*.md'):
        shutil.copy2(p,archive/p.name)
    shutil.copytree(DESKTOP/'验证截图',archive/'验证截图')
    print(json.dumps(dict(compared=len(pairs),equal=sum(x['equal'] for x in pairs),output=str(destination)),ensure_ascii=False))


def package(output):
    """Freeze canonical source/model plus evidence; do not replace old reports."""
    output=Path(output)
    environment=runtime_manifest()
    write_json(output/'environment.json',environment)
    write_json(output/'runtime_fingerprint.json',fingerprint(environment))
    versions=environment['packages']
    (output/'requirements-reproduce.txt').write_text('\n'.join(name+'=='+version for name,version in versions.items() if version)+'\n',encoding='utf-8')
    snapshot=output/'源码快照/slaihand_ros2_ws'
    files=set(runtime_files())
    files.update(ROOT/p for p in ('src/openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp',
                                  'src/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp'))
    files.add(ROOT/'verification/hardware/j3_return_reference.py')
    files.add(ROOT/'verification/hardware/commissioning_operator_confirmations.json')
    files.update((ROOT/'slai_console').glob('*.py'))
    files.update((ROOT/'slai_console').glob('*.md'))
    files.update((ROOT/'desktop_entries/03_桌面上位机').glob('*'))
    files.update((ROOT/'verification/hardware').glob('test_*.py'))
    inventory={}
    for source in sorted(files):
        if not source.is_file():
            continue
        relative=source.relative_to(ROOT)
        dest=snapshot/relative
        dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,dest)
        inventory[str(relative).replace('\\','/')]=sha(source)
        if sha(dest)!=sha(source):
            raise RuntimeError('Snapshot copy mismatch: '+str(relative))
    status=git('status','--porcelain=v1')
    write_json(output/'source_manifest.json',dict(at=now(),base_commit=git('rev-parse','HEAD'),
        dirty=bool(status),working_tree=status,files=inventory))
    diff=git('diff','--binary','HEAD','--','src/slaihand_bringup/config/hardware_mapping.yaml')
    (output/'tracked_changes.patch').write_text(diff+'\n' if diff else '',encoding='utf-8')
    env_lines=['# 环境与版本','',
        '记录时区：Asia/Shanghai；生成于 '+now()+'。此文件描述当前运行环境，不是硬件验收证据。','',
        '| 项目 | 实际值 |','|---|---|',
        '| Python | '+environment['python']+' |',
        '| Python 路径 | '+environment['executable']+' |',
        '| 操作系统 | '+environment['platform']+' |',
        '| PySide6 模块 | '+environment['pyside6_runtime']+' |',
        '| Qt 运行时 | '+environment['qt_runtime']+' |',
        '| MuJoCo | '+environment['mujoco_runtime']+' |',
        '| MuJoCo 路径 | '+environment['mujoco_path']+' |',
        '| Git 基线 | '+git('rev-parse','HEAD')+' |',
        '| 工作树 | '+('含未提交修改及新增文件；不能只 checkout 基线还原' if status else '干净')+' |','',
        '## 依赖记录','',
        '```text',*(name+' = '+str(v) for name,v in versions.items()),'```','',
        '包元数据缺失以 null 记录，不据此认定模块未安装；例如实际 Qt 模块来自 PySide6_Essentials。',
        'requirements-reproduce.txt 固定已检测到的包版本；未打包 Python、第三方二进制和显卡驱动。',
        '截图像素会受操作系统、缩放和图形驱动影响，不以跨机器逐像素相等作为功能判据。','',
        '## 模型与运行源码 SHA-256','',
        '| 文件 | SHA-256 |','|---|---|',
        *(f'| {p} | {h} |' for p,h in fingerprint(environment)['inputs']['files'].items()),'',
        '完整运行指纹：`'+fingerprint(environment)['sha256']+'`。','',
        '## 复现方式','',
        '桌面现有启动器仍指向当前仓库；源码快照不会自动替换它。源码快照是本控制台及模型所需的最小源码集合，不是整个 ROS2 工程备份。',
        '在 Python 3.12.10 环境安装 requirements-reproduce.txt 后，在快照的 slaihand_ros2_ws 目录运行：','',
        '```text','python -m slai_console.regression --output tmp/reproduce',
        'python -m unittest slai_console.test_console slai_console.test_commissioning slai_console.test_timing -v',
        'python -m unittest discover -s verification/hardware -p "test_*.py" -v',
        'python -m slai_console.app','```','',
        'source_manifest.json 绑定快照全部文件；tracked_changes.patch 仅补充已跟踪映射文件的改动，不能替代新增源码快照。','']
    (output/'环境与版本.md').write_text('\n'.join(env_lines),encoding='utf-8')
    # Excludes this manifest itself to avoid a self-referential hash.
    artifacts={str(p.relative_to(output)).replace('\\','/'):sha(p) for p in output.rglob('*') if p.is_file() and p.name!='artifact_manifest.json'}
    write_json(output/'artifact_manifest.json',dict(at=now(),files=artifacts))
    print(json.dumps(dict(snapshot_files=len(inventory),artifacts=len(artifacts),fingerprint=fingerprint(environment)['sha256']),ensure_ascii=False))


def verify(output):
    output=Path(output).resolve()
    manifest=json.loads((output/'artifact_manifest.json').read_text(encoding='utf-8'))
    for relative,expected in manifest['files'].items():
        path=(output/relative).resolve()
        if not path.is_relative_to(output) or not path.is_file() or sha(path)!=expected:
            raise RuntimeError('Artifact hash mismatch: '+relative)
    source=json.loads((output/'source_manifest.json').read_text(encoding='utf-8'))
    for relative,expected in source['files'].items():
        if sha(ROOT/relative)!=expected:
            raise RuntimeError('Canonical source has changed since snapshot: '+relative)
    expected=json.loads((output/'runtime_fingerprint.json').read_text(encoding='utf-8'))
    if fingerprint()!=expected:
        raise RuntimeError('Current runtime differs from frozen environment/model/source')
    print(json.dumps(dict(artifacts_checked=len(manifest['files']),source_files_checked=len(source['files']),
                          current_runtime_matches=True),ensure_ascii=False))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['inspect','fingerprint','package','verify'])
    parser.add_argument('--output',type=Path,default=ROOT/'tmp/console_release/2026-10-31')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    if args.action=='inspect':
        inspect_delivery(args.output)
    elif args.action=='package':
        package(args.output)
    elif args.action=='verify':
        verify(args.output)
    else:
        write_json(args.output/'capture_inputs_before.json',dict(at=now(),fingerprint=fingerprint()))


if __name__=='__main__':
    main()
