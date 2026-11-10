"""Compile an offline reviewer executable with the candidate's recorded flags."""
from pathlib import Path
import shlex
import subprocess
root=Path(__file__).resolve().parent
build=Path('/home/xuyu/slaihand_control_panel_ws_20261104/build/slaihand_hardware')
flags=(build/'CMakeFiles/motion_supervisor_test.dir/flags.make').read_text()
includes=next(row.split(' = ',1)[1] for row in flags.splitlines() if row.startswith('CXX_INCLUDES = '))
link=shlex.split((build/'CMakeFiles/motion_supervisor_test.dir/link.txt').read_text())
link[1]=str(root/'codex_b_repro.cpp')
link[link.index('-o')+1]='/tmp/codex_b_repro'
link[1:1]=['-std=c++17']+shlex.split(includes)
subprocess.run(link,cwd=build,check=True)
subprocess.run(['/tmp/codex_b_repro'],check=True)
