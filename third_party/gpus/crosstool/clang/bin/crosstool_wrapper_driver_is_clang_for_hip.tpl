#!/usr/bin/env python
"""Crosstool wrapper for compiling ROCm programs.

SYNOPSIS:
  crosstool_wrapper_driver_is_clang_for_hip [options passed in by cc_library()
                                or cc_binary() rule]

DESCRIPTION:
  This script is expected to be called by the cc_library() or cc_binary() bazel
  rules. When the option "-x hip" is present in the list of arguments passed
  to this script, it invokes the hipcc compiler. Most arguments are passed
  as is as a string to --compiler-options of hipcc. When "-x hip" is not
  present, this wrapper invokes gcc with the input arguments as is.
"""

from argparse import ArgumentParser
import os
import subprocess
import re
import sys
import pipes
import base64

# Template values set by rocm_configure.bzl.
CPU_COMPILER = ('%{cpu_compiler}')

HIPCC_PATH = '%{hipcc_path}'
HIPCC_ENV = '%{hipcc_env}'
HIP_RUNTIME_PATH = '%{hip_runtime_path}'
HIP_RUNTIME_LIBRARY = '%{hip_runtime_library}'
ROCR_RUNTIME_PATH = '%{rocr_runtime_path}'
ROCR_RUNTIME_LIBRARY = '%{rocr_runtime_library}'
VERBOSE = %{crosstool_verbose}

def Log(s):
  print('gpus/crosstool: {0}'.format(s))


def GetOptionValue(argv, option):
  """Extract the list of values for option from the argv list.

  Args:
    argv: A list of strings, possibly the argv passed to main().
    option: The option whose value to extract, with the leading '-'.

  Returns:
    A list of values, either directly following the option,
    (eg., -opt val1 val2) or values collected from multiple occurrences of
    the option (eg., -opt val1 -opt val2).
  """

  parser = ArgumentParser()
  parser.add_argument(option, nargs='*', action='append')
  option = option.lstrip('-').replace('-', '_')
  args, _ = parser.parse_known_args(argv)
  if not args or not vars(args)[option]:
    return []
  else:
    return sum(vars(args)[option], [])


def GetHostCompilerOptions(argv):
  """Collect the -isystem, -iquote, and --sysroot option values from argv.

  Args:
    argv: A list of strings, possibly the argv passed to main().

  Returns:
    The string that can be used as the --compiler-options to hipcc.
  """

  parser = ArgumentParser()
  parser.add_argument('-isystem', nargs='*', action='append')
  parser.add_argument('-iquote', nargs='*', action='append')
  parser.add_argument('--sysroot', nargs=1)
  parser.add_argument('-g', nargs='*', action='append')
  parser.add_argument('-fno-canonical-system-headers', action='store_true')

  args, _ = parser.parse_known_args(argv)

  opts = ''

  if args.isystem:
    opts += ' -isystem ' + ' -isystem '.join(sum(args.isystem, []))
  if args.iquote:
    opts += ' -iquote ' + ' -iquote '.join(sum(args.iquote, []))
  if args.g:
    opts += ' -g' + ' -g'.join(sum(args.g, []))
  if args.sysroot:
    opts += ' --sysroot ' + args.sysroot[0]

  return opts

def system(cmd):
  """Invokes cmd with os.system().

  Args:
    cmd: The command.

  Returns:
    The exit code if the process exited with exit() or -signal
    if the process was terminated by a signal.
  """
  retv = os.system(cmd)
  if os.WIFEXITED(retv):
    return os.WEXITSTATUS(retv)
  else:
    return -os.WTERMSIG(retv)


def InvokeHipcc(argv, log=False):
  """Call hipcc with arguments assembled from argv.

  Args:
    argv: A list of strings, possibly the argv passed to main().
    log: True if logging is requested.

  Returns:
    The return value of calling os.system('hipcc ' + args)
  """

  host_compiler_options = GetHostCompilerOptions(argv)
  opt_option = GetOptionValue(argv, '-O')
  m_options = GetOptionValue(argv, '-m')
  m_options = ''.join([' -m' + m for m in m_options if m in ['32', '64']])
  include_options = GetOptionValue(argv, '-I')
  out_file = GetOptionValue(argv, '-o')
  depfiles = GetOptionValue(argv, '-MF')
  defines = GetOptionValue(argv, '-D')
  defines = ''.join([' -D' + define for define in defines])
  undefines = GetOptionValue(argv, '-U')
  undefines = ''.join([' -U' + define for define in undefines])
  std_options = GetOptionValue(argv, '-std')
  hipcc_allowed_std_options = ["c++11", "c++14"]
  std_options = ''.join([' -std=' + define
      for define in std_options if define in hipcc_allowed_std_options])

  # The list of source files get passed after the -c option.
  src_files = GetOptionValue(argv, '-c')

  if len(src_files) == 0:
    return 1
  if len(out_file) != 1:
    return 1

  opt = (' -O2' if (len(opt_option) > 0 and int(opt_option[0]) > 0)
         else ' -g')

  includes = (' -I ' + ' -I '.join(include_options)
              if len(include_options) > 0
              else '')

  # Unfortunately, there are other options that have -c prefix too.
  # So allowing only those look like C/C++ files.
  src_files = [f for f in src_files if
               re.search('\.cpp$|\.cc$|\.c$|\.cxx$|\.C$|\.cu$', f)]
  srcs = ' '.join(src_files)
  out = ' -o ' + out_file[0]

  # We need to make sure that the hip header is included in the sources before
  # any standard math header like <complex>. Otherwise, we get a build error.
  # Also we need to retain warnings about uninitialised shared variables as
  # 'warning only', even if the '-Werror' option is specified.
  hipccopts = '--include=hip/hip_runtime.h'

  if VERBOSE: hipccopts += ' -v'

  for offload in GetOptionValue(argv, '--offload-arch'):
    hipccopts += ''.join([' --offload-arch=' + offload])

  # Use '-fno-gpu-rdc' by default for early GPU kernel finalization.
  # This flag will trigger GPU kernels to be generated at compile time instead
  # of link time. This allows the default host compiler (gcc) to be used as the
  # linker for TensorFlow on the ROCm platform.
  hipccopts += ' -fno-gpu-rdc'

  hipccopts += undefines
  hipccopts += defines
  hipccopts += std_options
  hipccopts += m_options

  hipccopts_override = ' -Xclang -Wno-deprecated-declarations'
  hipccopts_override += ' -fcolor-diagnostics'

  if depfiles:
    # Generate the dependency file
    depfile = depfiles[0]
    cmd = (HIPCC_PATH + ' ' + hipccopts +
           host_compiler_options + hipccopts_override +
           ' -I .' + includes + ' ' + srcs + ' -M -o ' + depfile)
    cmd = HIPCC_ENV.replace(';', ' ') + ' ' + cmd
    if log: Log(cmd)
    if VERBOSE:
      print('  HIPCC=' + HIPCC_ENV)
      print(cmd)
    exit_status = os.system(cmd)
    if exit_status != 0:
      return exit_status

  cmd = (HIPCC_PATH + ' ' + hipccopts +
         host_compiler_options + hipccopts_override + ' -fPIC' +
         ' -I .' + opt + includes + ' -c ' + srcs + out)

  cmd = HIPCC_ENV.replace(';', ' ') + ' ' + cmd
  if log: Log(cmd)
  if VERBOSE:
    print('  HIPCC=' + HIPCC_ENV)
    print(cmd)

  return system(cmd)


def main():
  # Ignore PWD env var
  os.environ['PWD']=''

  parser = ArgumentParser(fromfile_prefix_chars='@')
  parser.add_argument('-x', nargs=1)
  parser.add_argument('--rocm_log', action='store_true')
  parser.add_argument('-pass-exit-codes', action='store_true')
  args, leftover = parser.parse_known_args(sys.argv[1:])

  if VERBOSE: print('PWD=' + os.getcwd())

  if args.x and args.x[0] == 'hip':
    # Compilation of GPU objects
    if args.rocm_log: Log('-x hip')
    leftover = [pipes.quote(s) for s in leftover]
    if args.rocm_log: Log('using hipcc')
    return InvokeHipcc(leftover, log=args.rocm_log)

  elif args.pass_exit_codes:
    # Link with hipcc compiler invoked with '-fno-gpu-rdc' by default.
    # Host compiler can be used as a linker,
    # but HIP runtime libraries should be specified.
    gpu_linker_flags = [flag for flag in sys.argv[1:]
                               if not flag.startswith(('--rocm_log'))]
    gpu_linker_flags.append('-L' + ROCR_RUNTIME_PATH)
    gpu_linker_flags.append('-Wl,-rpath=' + ROCR_RUNTIME_PATH)
    gpu_linker_flags.append('-l' + ROCR_RUNTIME_LIBRARY)
    gpu_linker_flags.append('-L' + HIP_RUNTIME_PATH)
    gpu_linker_flags.append('-Wl,-rpath=' + HIP_RUNTIME_PATH)
    gpu_linker_flags.append('-l' + HIP_RUNTIME_LIBRARY)
    gpu_linker_flags.append("-lrt")
    gpu_linker_flags.append("-lstdc++")
    if VERBOSE:
      print('  LD=')
      print(' '.join([CPU_COMPILER] + gpu_linker_flags))
    return subprocess.call([CPU_COMPILER] + gpu_linker_flags)

  else:
    # Compilation of host objects
    # Strip flags before passing through to the CPU compiler for files which
    # are not '-x hip'. We can't just pass 'leftover' because it also strips -x.
    # We not only want to pass -x to the CPU compiler but also to keep it in its
    # relative location in the argv list (the compiler is sensitive to this).
    cpu_compiler_flags = [flag for flag in sys.argv[1:]
                               if not flag.startswith(('--rocm_log'))]

    # SE codes need to be built with gcc, but need this macro defined.
    # From ROCm HIP's hip_common.h:
    #   Auto enable __HIP_PLATFORM_AMD__ if compiling on AMD platform
    #   Other compiler (GCC,ICC,etc) need to set one of these macros explicitly
    cpu_compiler_flags.append("-D__HIP_PLATFORM_AMD__")
    cpu_compiler_flags.append("-Wno-deprecated-declarations")
    if VERBOSE:
      print('  GCC=')
      print(' '.join([CPU_COMPILER] + cpu_compiler_flags))
    return subprocess.call([CPU_COMPILER] + cpu_compiler_flags)

if __name__ == '__main__':
  sys.exit(main())
