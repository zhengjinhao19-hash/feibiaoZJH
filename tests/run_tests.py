"""Build actual portable C++ implementation with a native (not ARM) compiler."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--cxx', default=os.environ.get('HOST_CXX') or shutil.which('g++'))
parser.add_argument('--output', type=Path)
parser.add_argument('--mingw-root', type=Path, help='Optional MinGW sysroot for a Clang host compiler')
args = parser.parse_args()
if not args.cxx:
    parser.error('Provide --cxx <native g++.exe path> or set HOST_CXX. ARM GCC cannot run host tests.')
root = Path(__file__).resolve().parents[1]
out = args.output or root / 'build' / 'host-tests'
out.mkdir(parents=True, exist_ok=True)
env = dict(os.environ)
env['PATH'] = str(Path(args.cxx).resolve().parent) + os.pathsep + env.get('PATH', '')
compiler_flags = []
if args.mingw_root:
    env['PATH'] = str(args.mingw_root / 'bin') + os.pathsep + env['PATH']
    compiler_flags = ['--driver-mode=g++', '--target=x86_64-w64-windows-gnu',
                      '--sysroot=' + str(args.mingw_root), '-fsjlj-exceptions']
common = [args.cxx] + compiler_flags + ['-std=c++17', '-Wall', '-Wextra', '-Werror', '-O1', '-g',
          '-I' + str(root / 'inc'), '-I' + str(root / 'Instance'), '-I' + str(root / 'Task')]
suffix = '.exe' if os.name == 'nt' else ''

def run(command):
    print('RUN:', ' '.join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), check=True, env=env)

core = out / ('test_core' + suffix)
run(common + [root / 'tests/test_core.cpp', root / 'src/diagnostic_core.cpp',
              root / 'src/DT7.cpp', '-o', core])
run([core, out / 'pid_simulation.csv'])
runtime = out / ('test_runtime' + suffix)
run(common + ['-I' + str(root / 'tests/fakes'), root / 'tests/test_runtime.cpp',
              root / 'src/diagnostic_core.cpp', root / 'src/DT7.cpp',
              root / 'Instance/rc_ins.cpp', root / 'Task/comm_task.cpp',
              root / 'Task/main_task.cpp', '-o', runtime])
for scenario in ('normal', 'init_failure', 'timer_failure', 'arm_failure'):
    run([runtime, scenario])
print('All portable and mocked-HAL tests passed. Hardware is not validated.', flush=True)
