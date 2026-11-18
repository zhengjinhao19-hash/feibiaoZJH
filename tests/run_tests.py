"""Build actual portable C++ implementation with a native (not ARM) compiler."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--cxx', default=os.environ.get('HOST_CXX') or shutil.which('g++'))
parser.add_argument('--output', type=Path)
parser.add_argument('--static-runtime', action='store_true', help='Statically link MinGW runtime DLLs')
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
if args.static_runtime:
    compiler_flags += ['-static', '-static-libgcc', '-static-libstdc++']

include_dirs = [
    root / 'inc',
    root / 'Instance',
    root / 'Task',
    root / 'Dart-Components',
    root / 'Dart-Components' / 'fsm',
    root / 'Dart-Components' / 'modules' / 'belt',
    root / 'Dart-Components' / 'modules' / 'screw',
    root / 'Dart-Components' / 'modules' / 'shoot',
    root / 'Dart-Configs',
]
common = [args.cxx] + compiler_flags + [
    '-std=c++17', '-Wall', '-Wextra', '-Werror', '-O1', '-g',
    '-Wno-unknown-pragmas', '-Wno-unused-parameter',
    '-Wno-missing-field-initializers',
    '-DSTM32_HAL_FILENAME=<stm32f4xx_hal.h>',
] + ['-I' + str(path) for path in include_dirs]
suffix = '.exe' if os.name == 'nt' else ''

def run(command):
    print('RUN:', ' '.join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), check=True, env=env)

core = out / ('test_core' + suffix)
run(common + [root / 'tests/test_core.cpp', root / 'src/diagnostic_core.cpp',
              root / 'src/DT7.cpp', '-o', core])
run([core, out / 'pid_simulation.csv'])

fake_include = ['-I' + str(root / 'tests' / 'fakes')]
control = out / ('test_control' + suffix)
run(common + fake_include + [root / 'tests/test_control.cpp',
                             root / 'src/motor_device.cpp',
                             root / 'src/diagnostic_core.cpp',
                             root / 'src/DT7.cpp', '-o', control])
run([control])

runtime = out / ('test_runtime' + suffix)
runtime_sources = [
    root / 'tests/test_runtime.cpp',
    root / 'src/diagnostic_core.cpp',
    root / 'src/DT7.cpp',
    root / 'src/motor_device.cpp',
    root / 'src/servo.cpp',
    root / 'Instance/rc_ins.cpp',
    root / 'Instance/motor_ins.cpp',
    root / 'Instance/fsm_ins.cpp',
    root / 'Task/comm_task.cpp',
    root / 'Task/main_task.cpp',
    root / 'Dart-Components/modules/belt/belt_fsm.cpp',
    root / 'Dart-Components/modules/screw/screw_fsm.cpp',
    root / 'Dart-Components/modules/shoot/shoot_fsm.cpp',
    root / 'Dart-Components/fsm/main_fsm.cpp',
]
run(common + fake_include + runtime_sources + ['-o', runtime])
for scenario in ('normal', 'init_failure', 'timer_failure', 'arm_failure',
                 'can_start_failure', 'notification_failure', 'pwm_failure',
                 'active_busoff', 'active_tx_failure', 'active_no_ack',
                 'active_can_error', 'active_motor_loss', 'active_uart_error',
                 'manual_active_busoff', 'manual_active_tx_failure', 'manual_active_no_ack',
                 'manual_active_can_error', 'manual_active_motor_loss', 'manual_active_uart_error'):
    run([runtime, scenario])

sequence = out / ('test_sequence' + suffix)
sequence_sources = [
    root / 'tests/test_sequence.cpp',
    root / 'src/diagnostic_core.cpp',
    root / 'src/DT7.cpp',
    root / 'src/motor_device.cpp',
    root / 'src/servo.cpp',
    root / 'Dart-Components/modules/belt/belt_fsm.cpp',
    root / 'Dart-Components/modules/screw/screw_fsm.cpp',
    root / 'Dart-Components/modules/shoot/shoot_fsm.cpp',
    root / 'Dart-Components/fsm/main_fsm.cpp',
]
run(common + fake_include + sequence_sources + ['-o', sequence])
run([sequence])
safety = out / ('test_safety' + suffix)
run(common + fake_include + [root / 'tests/test_safety.cpp'] + sequence_sources[1:] + ['-o', safety])
run([safety])
manual = out / ('test_manual' + suffix)
run(common + fake_include + [root / 'tests/test_manual.cpp'] + sequence_sources[1:] + ['-o', manual])
run([manual])
relief = out / ('test_relief' + suffix)
run(common + fake_include + [root / 'tests/test_relief.cpp'] + sequence_sources[1:] + ['-o', relief])
run([relief])

print('All portable and mocked-HAL tests passed. Hardware is not validated.', flush=True)
