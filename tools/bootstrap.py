#! /usr/bin/env python3
# Adapted from tensorflow.git/configure.py
# Copyright 2017 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""configure script to get build parameters from user."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import errno
import os
import platform
import re
import subprocess
import sys

# pylint: disable=g-import-not-at-top
try:
    from shutil import which
except ImportError:
    from distutils.spawn import find_executable as which
# pylint: enable=g-import-not-at-top

_DEFAULT_CUDA_VERSION = '10'
_DEFAULT_CUDNN_VERSION = '7'
_DEFAULT_TENSORRT_VERSION = '7'
_DEFAULT_CUDA_COMPUTE_CAPABILITIES = '6.0,6.1,7.0,7.2,7.5'

_DEFAULT_PROMPT_ASK_ATTEMPTS = 10

_TF_WORKSPACE_ROOT = ''
_TF_BAZELRC = ''
_TF_CURRENT_BAZEL_VERSION = None
_TF_MIN_BAZEL_VERSION = '2.0.0'
_TF_MAX_BAZEL_VERSION = '3.99.0'

class UserInputError(Exception):
    pass


def is_linux():
    return platform.system() == 'Linux'

def get_input(question):
    try:
        try:
            answer = raw_input(question)
        except NameError:
            answer = input(question)  # pylint: disable=bad-builtin
    except EOFError:
        answer = ''
    return answer


def symlink_force(target, link_name):
    """Force symlink, equivalent of 'ln -sf'.

  Args:
    target: items to link to.
    link_name: name of the link.
  """
    try:
        os.symlink(target, link_name)
    except OSError as e:
        if e.errno == errno.EEXIST:
            os.remove(link_name)
            os.symlink(target, link_name)
        else:
            raise e


def sed_in_place(filename, old, new):
    """Replace old string with new string in file.

  Args:
    filename: string for filename.
    old: string to replace.
    new: new string to replace to.
  """
    with open(filename, 'r') as f:
        filedata = f.read()
    newdata = filedata.replace(old, new)
    with open(filename, 'w') as f:
        f.write(newdata)


def write_to_bazelrc(line):
    with open(_TF_BAZELRC, 'a') as f:
        f.write(line + '\n')

def write_action_env_to_bazelrc(var_name, var):
    write_to_bazelrc('build --action_env {}="{}"'.format(var_name, str(var)))

def run_shell(cmd, allow_non_zero=False, stderr=None):
    if stderr is None:
        stderr = sys.stdout
    if allow_non_zero:
        try:
            output = subprocess.check_output(cmd, stderr=stderr)
        except subprocess.CalledProcessError as e:
            output = e.output
    else:
        output = subprocess.check_output(cmd, stderr=stderr)
    return output.decode('UTF-8').strip()

def get_python_path(environ_cp, python_bin_path):
    """Get the python site package paths."""
    python_paths = []
    if environ_cp.get('PYTHONPATH'):
        python_paths = environ_cp.get('PYTHONPATH').split(':')
    try:
        stderr = open(os.devnull, 'wb')
        library_paths = run_shell(
            [
                python_bin_path, '-c',
                'import site; print("\\n".join(site.getsitepackages()))'
            ],
            stderr=stderr).split('\n')
    except subprocess.CalledProcessError:
        library_paths = [
            run_shell([
                python_bin_path, '-c',
                'from distutils.sysconfig import get_python_lib;'
                'print(get_python_lib())'
            ])
        ]

    all_paths = set(python_paths + library_paths)

    paths = []
    for path in all_paths:
        if os.path.isdir(path):
            paths.append(path)
    return paths

def get_python_major_version(python_bin_path):
    """Get the python major version."""
    return run_shell(
        [python_bin_path, '-c', 'import sys; print(sys.version[0])'])

def setup_python(environ_cp):
    """Setup python related env variables."""
    # Get PYTHON_BIN_PATH, default is the current running python.
    default_python_bin_path = sys.executable
    ask_python_bin_path = (
        'Please specify the location of python. [Default is '
        '{}]: ').format(default_python_bin_path)
    while True:
        python_bin_path = get_from_env_or_user_or_default(
            environ_cp, 'PYTHON_BIN_PATH', ask_python_bin_path,
            default_python_bin_path)
        # Check if the path is valid
        if os.path.isfile(python_bin_path) and os.access(
                python_bin_path, os.X_OK):
            break
        elif not os.path.exists(python_bin_path):
            print('Invalid python path: {} cannot be found.'.format(
                python_bin_path))
        else:
            print('{} is not executable.  Is it the python binary?'.format(
                python_bin_path))
        environ_cp['PYTHON_BIN_PATH'] = ''

    # Get PYTHON_LIB_PATH
    python_lib_path = environ_cp.get('PYTHON_LIB_PATH')
    if not python_lib_path:
        python_lib_paths = get_python_path(environ_cp, python_bin_path)
        if environ_cp.get('USE_DEFAULT_PYTHON_LIB_PATH') == '1':
            python_lib_path = python_lib_paths[0]
        else:
            print('Found possible Python library paths:\n  %s' %
                  '\n  '.join(python_lib_paths))
            default_python_lib_path = python_lib_paths[0]
            python_lib_path = get_input(
                'Please input the desired Python library path to use.  '
                'Default is [{}]\n'.format(python_lib_paths[0]))
            if not python_lib_path:
                python_lib_path = default_python_lib_path
        environ_cp['PYTHON_LIB_PATH'] = python_lib_path

    python_major_version = get_python_major_version(python_bin_path)
    if python_major_version != '3':
        print('Python 2 was RETIRED on April 2020. Use Python 3 instead.')
        sys.exit(1)

    # Set-up env variables used by python_configure.bzl
    write_action_env_to_bazelrc('PYTHON_BIN_PATH', python_bin_path)
    write_action_env_to_bazelrc('PYTHON_LIB_PATH', python_lib_path)
    write_to_bazelrc('build --python_path=\"{}"'.format(python_bin_path))
    environ_cp['PYTHON_BIN_PATH'] = python_bin_path

    # If choosen python_lib_path is from a path specified in the PYTHONPATH
    # variable, need to tell bazel to include PYTHONPATH
    if environ_cp.get('PYTHONPATH'):
        python_paths = environ_cp.get('PYTHONPATH').split(':')
        if python_lib_path in python_paths:
            write_action_env_to_bazelrc('PYTHONPATH',
                                        environ_cp.get('PYTHONPATH'))
    # Write tools/python_bin_path.sh
    with open(
            os.path.join(_TF_WORKSPACE_ROOT, 'tools', 'python_bin_path.sh'),
            'w') as f:
        f.write('export PYTHON_BIN_PATH="{}"'.format(python_bin_path))

def reset_tf_configure_bazelrc():
    """Reset file that contains customized config settings."""
    open(_TF_BAZELRC, 'w').close()

def get_var(environ_cp,
            var_name,
            query_item,
            enabled_by_default,
            question=None,
            yes_reply=None,
            no_reply=None):
    """Get boolean input from user.

  If var_name is not set in env, ask user to enable query_item or not. If the
  response is empty, use the default.

  Args:
    environ_cp: copy of the os.environ.
    var_name: string for name of environment variable, e.g. "TF_NEED_CUDA".
    query_item: string for feature related to the variable, e.g. "CUDA for
      Nvidia GPUs".
    enabled_by_default: boolean for default behavior.
    question: optional string for how to ask for user input.
    yes_reply: optional string for reply when feature is enabled.
    no_reply: optional string for reply when feature is disabled.

  Returns:
    boolean value of the variable.

  Raises:
    UserInputError: if an environment variable is set, but it cannot be
      interpreted as a boolean indicator, assume that the user has made a
      scripting error, and will continue to provide invalid input.
      Raise the error to avoid infinitely looping.
  """
    if not question:
        question = 'Do you wish to build your project with {} support?'.format(
            query_item)
    if not yes_reply:
        yes_reply = '{} support will be enabled for your project.'.format(
            query_item)
    if not no_reply:
        no_reply = 'No {}'.format(yes_reply)

    yes_reply += '\n'
    no_reply += '\n'

    if enabled_by_default:
        question += ' [Y/n]: '
    else:
        question += ' [y/N]: '

    var = environ_cp.get(var_name)
    if var is not None:
        var_content = var.strip().lower()
        true_strings = ('1', 't', 'true', 'y', 'yes')
        false_strings = ('0', 'f', 'false', 'n', 'no')
        if var_content in true_strings:
            var = True
        elif var_content in false_strings:
            var = False
        else:
            raise UserInputError(
                'Environment variable %s must be set as a boolean indicator.\n'
                'The following are accepted as TRUE : %s.\n'
                'The following are accepted as FALSE: %s.\n'
                'Current value is %s.' % (var_name, ', '.join(true_strings),
                                          ', '.join(false_strings), var))

    while var is None:
        user_input_origin = get_input(question)
        user_input = user_input_origin.strip().lower()
        if user_input == 'y':
            print(yes_reply)
            var = True
        elif user_input == 'n':
            print(no_reply)
            var = False
        elif not user_input:
            if enabled_by_default:
                print(yes_reply)
                var = True
            else:
                print(no_reply)
                var = False
        else:
            print('Invalid selection: {}'.format(user_input_origin))
    return var


def set_build_var(environ_cp,
                  var_name,
                  query_item,
                  option_name,
                  enabled_by_default,
                  bazel_config_name=None):
    """Set if query_item will be enabled for the build.

  Ask user if query_item will be enabled. Default is used if no input is given.
  Set subprocess environment variable and write to .bazelrc if enabled.

  Args:
    environ_cp: copy of the os.environ.
    var_name: string for name of environment variable, e.g. "TF_NEED_CUDA".
    query_item: string for feature related to the variable, e.g. "CUDA for
      Nvidia GPUs".
    option_name: string for option to define in .bazelrc.
    enabled_by_default: boolean for default behavior.
    bazel_config_name: Name for Bazel --config argument to enable build feature.
  """

    var = str(
        int(get_var(environ_cp, var_name, query_item, enabled_by_default)))
    environ_cp[var_name] = var
    if var == '1':
        write_to_bazelrc('build:%s --define %s=true' % (bazel_config_name,
                                                        option_name))
        write_to_bazelrc('build --config=%s' % bazel_config_name)
    elif bazel_config_name is not None:
        # TODO(mikecase): Migrate all users of configure.py to use --config Bazel
        # options and not to set build configs through environment variables.
        write_to_bazelrc('build:%s --define %s=true' % (bazel_config_name,
                                                        option_name))


def set_action_env_var(environ_cp,
                       var_name,
                       query_item,
                       enabled_by_default,
                       question=None,
                       yes_reply=None,
                       no_reply=None,
                       bazel_config_name=None):
    """Set boolean action_env variable.

  Ask user if query_item will be enabled. Default is used if no input is given.
  Set environment variable and write to .bazelrc.

  Args:
    environ_cp: copy of the os.environ.
    var_name: string for name of environment variable, e.g. "TF_NEED_CUDA".
    query_item: string for feature related to the variable, e.g. "CUDA for
      Nvidia GPUs".
    enabled_by_default: boolean for default behavior.
    question: optional string for how to ask for user input.
    yes_reply: optional string for reply when feature is enabled.
    no_reply: optional string for reply when feature is disabled.
    bazel_config_name: adding config to .bazelrc instead of action_env.
  """
    var = int(
        get_var(environ_cp, var_name, query_item, enabled_by_default, question,
                yes_reply, no_reply))

    if not bazel_config_name:
        write_action_env_to_bazelrc(var_name, var)
    elif var:
        write_to_bazelrc('build --config=%s' % bazel_config_name)
    environ_cp[var_name] = str(var)


def convert_version_to_int(version):
    """Convert a version number to a integer that can be used to compare.

  Version strings of the form X.YZ and X.Y.Z-xxxxx are supported. The
  'xxxxx' part, for instance 'homebrew' on OS/X, is ignored.

  Args:
    version: a version to be converted

  Returns:
    An integer if converted successfully, otherwise return None.
  """
    version = version.split('-')[0]
    version_segments = version.split('.')
    # Treat "0.24" as "0.24.0"
    if len(version_segments) == 2:
        version_segments.append('0')
    for seg in version_segments:
        if not seg.isdigit():
            return None

    version_str = ''.join(['%03d' % int(seg) for seg in version_segments])
    return int(version_str)


def check_bazel_version(min_version, max_version):
    """Check installed bazel version is between min_version and max_version.

    Args:
        min_version: string for minimum bazel version (must exist!).
        max_version: string for maximum bazel version (must exist!).

    Returns:
        The bazel version detected.
    """
    if which('bazel') is None:
        print('Cannot find bazel. Please install bazel.')
        sys.exit(0)

    stderr = open(os.devnull, 'wb')
    curr_version = run_shell(
        ['bazel', '--version'], allow_non_zero=True, stderr=stderr)
    if curr_version.startswith('bazel '):
        curr_version = curr_version.split('bazel ')[1]

    min_version_int = convert_version_to_int(min_version)
    curr_version_int = convert_version_to_int(curr_version)
    max_version_int = convert_version_to_int(max_version)

    # Check if current bazel version can be detected properly.
    if not curr_version_int:
        print('WARNING: current bazel installation is not a release version.')
        print('Make sure you are running at least bazel %s' % min_version)
        return curr_version

    print('You have bazel %s installed.' % curr_version)

    if curr_version_int < min_version_int:
        print(
            'Please upgrade your bazel installation to version %s or higher' % min_version)
        sys.exit(1)
    if (curr_version_int > max_version_int
            and 'TF_IGNORE_MAX_BAZEL_VERSION' not in os.environ):
        print(
            'Please downgrade your bazel installation to version %s or lower'
            '! To downgrade: download the installer for the old version (from'
            'https://github.com/bazelbuild/bazel/releases) then run the '
            'installer.' % max_version)
        sys.exit(1)
    return curr_version


def set_cc_opt_flags(environ_cp):
    """Set up architecture-dependent optimization flags.

  Also append CC optimization flags to bazel.rc..

  Args:
    environ_cp: copy of the os.environ.
  """
    default_cc_opt_flags = '-march=native'
    question = (
        'Please specify optimization flags to use during compilation when'
        ' bazel option "--config=opt" is specified [Default is %s]: '
    ) % default_cc_opt_flags
    cc_opt_flags = get_from_env_or_user_or_default(
        environ_cp, 'CC_OPT_FLAGS', question, default_cc_opt_flags)
    for opt in cc_opt_flags.split():
        write_to_bazelrc('build:opt --copt=%s' % opt)
    # It should be safe on the same build host.
    write_to_bazelrc('build:opt --host_copt=-march=native')
    write_to_bazelrc('build:opt --define with_default_optimizations=true')


def set_tf_cuda_clang(environ_cp):
    """set TF_CUDA_CLANG action_env.

  Args:
    environ_cp: copy of the os.environ.
  """
    question = 'Do you want to use clang as CUDA compiler?'
    yes_reply = 'Clang will be used as CUDA compiler.'
    no_reply = 'nvcc will be used as CUDA compiler.'
    set_action_env_var(
        environ_cp,
        'TF_CUDA_CLANG',
        None,
        False,
        question=question,
        yes_reply=yes_reply,
        no_reply=no_reply,
        bazel_config_name='cuda_clang')

def get_from_env_or_user_or_default(environ_cp, var_name, ask_for_var,
                                    var_default):
    """Get var_name either from env, or user or default.

  If var_name has been set as environment variable, use the preset value, else
  ask for user input. If no input is provided, the default is used.

  Args:
    environ_cp: copy of the os.environ.
    var_name: string for name of environment variable, e.g. "TF_NEED_CUDA".
    ask_for_var: string for how to ask for user input.
    var_default: default value string.

  Returns:
    string value for var_name
  """
    var = environ_cp.get(var_name)
    if not var:
        var = get_input(ask_for_var)
        print('\n')
    if not var:
        var = var_default
    return var


def set_clang_cuda_compiler_path(environ_cp):
    """Set CLANG_CUDA_COMPILER_PATH."""
    default_clang_path = which('clang') or ''
    ask_clang_path = (
        'Please specify which clang should be used as device and '
        'host compiler. [Default is %s]: ') % default_clang_path

    while True:
        clang_cuda_compiler_path = get_from_env_or_user_or_default(
            environ_cp, 'CLANG_CUDA_COMPILER_PATH', ask_clang_path,
            default_clang_path)
        if os.path.exists(clang_cuda_compiler_path):
            break

        # Reset and retry
        print('Invalid clang path: %s cannot be found.' %
              clang_cuda_compiler_path)
        environ_cp['CLANG_CUDA_COMPILER_PATH'] = ''

    # Set CLANG_CUDA_COMPILER_PATH
    environ_cp['CLANG_CUDA_COMPILER_PATH'] = clang_cuda_compiler_path
    write_action_env_to_bazelrc('CLANG_CUDA_COMPILER_PATH',
                                clang_cuda_compiler_path)

def prompt_loop_or_load_from_env(environ_cp,
                                 var_name,
                                 var_default,
                                 ask_for_var,
                                 check_success,
                                 error_msg,
                                 suppress_default_error=False,
                                 resolve_symlinks=False,
                                 n_ask_attempts=_DEFAULT_PROMPT_ASK_ATTEMPTS):
    """Loop over user prompts for an ENV param until receiving a valid response.

  For the env param var_name, read from the environment or verify user input
  until receiving valid input. When done, set var_name in the environ_cp to its
  new value.

  Args:
    environ_cp: (Dict) copy of the os.environ.
    var_name: (String) string for name of environment variable, e.g. "TF_MYVAR".
    var_default: (String) default value string.
    ask_for_var: (String) string for how to ask for user input.
    check_success: (Function) function that takes one argument and returns a
      boolean. Should return True if the value provided is considered valid. May
      contain a complex error message if error_msg does not provide enough
      information. In that case, set suppress_default_error to True.
    error_msg: (String) String with one and only one '%s'. Formatted with each
      invalid response upon check_success(input) failure.
    suppress_default_error: (Bool) Suppress the above error message in favor of
      one from the check_success function.
    resolve_symlinks: (Bool) Translate symbolic links into the real filepath.
    n_ask_attempts: (Integer) Number of times to query for valid input before
      raising an error and quitting.

  Returns:
    [String] The value of var_name after querying for input.

  Raises:
    UserInputError: if a query has been attempted n_ask_attempts times without
      success, assume that the user has made a scripting error, and will
      continue to provide invalid input. Raise the error to avoid infinitely
      looping.
  """
    default = environ_cp.get(var_name) or var_default
    full_query = '%s [Default is %s]: ' % (
        ask_for_var,
        default,
    )

    for _ in range(n_ask_attempts):
        val = get_from_env_or_user_or_default(environ_cp, var_name, full_query,
                                              default)
        if check_success(val):
            break
        if not suppress_default_error:
            print(error_msg % val)
        environ_cp[var_name] = ''
    else:
        raise UserInputError(
            'Invalid %s setting was provided %d times in a row. '
            'Assuming to be a scripting mistake.' % (var_name, n_ask_attempts))

    if resolve_symlinks and os.path.islink(val):
        val = os.path.realpath(val)
    environ_cp[var_name] = val
    return val


def set_gcc_host_compiler_path(environ_cp):
    """Set GCC_HOST_COMPILER_PATH."""
    default_gcc_host_compiler_path = which('gcc') or ''
    cuda_bin_symlink = '%s/bin/gcc' % environ_cp.get('CUDA_TOOLKIT_PATH')

    if os.path.islink(cuda_bin_symlink):
        # os.readlink is only available in linux
        default_gcc_host_compiler_path = os.path.realpath(cuda_bin_symlink)

    gcc_host_compiler_path = prompt_loop_or_load_from_env(
        environ_cp,
        var_name='GCC_HOST_COMPILER_PATH',
        var_default=default_gcc_host_compiler_path,
        ask_for_var='Please specify which gcc should be used by nvcc as the host compiler.',
        check_success=os.path.exists,
        resolve_symlinks=True,
        error_msg='Invalid gcc path. %s cannot be found.',
    )

    write_action_env_to_bazelrc('GCC_HOST_COMPILER_PATH',
                                gcc_host_compiler_path)


def reformat_version_sequence(version_str, sequence_count):
    """Reformat the version string to have the given number of sequences.

  For example:
  Given (7, 2) -> 7.0
        (7.0.1, 2) -> 7.0
        (5, 1) -> 5
        (5.0.3.2, 1) -> 5

  Args:
      version_str: String, the version string.
      sequence_count: int, an integer.

  Returns:
      string, reformatted version string.
  """
    v = version_str.split('.')
    if len(v) < sequence_count:
        v = v + (['0'] * (sequence_count - len(v)))

    return '.'.join(v[:sequence_count])


def set_tf_cuda_paths(environ_cp):
    """Set TF_CUDA_PATHS."""
    ask_cuda_paths = (
        'Please specify the comma-separated list of base paths to look for CUDA '
        'libraries and headers. [Leave empty to use the default]: ')
    tf_cuda_paths = get_from_env_or_user_or_default(
        environ_cp, 'TF_CUDA_PATHS', ask_cuda_paths, '')
    if tf_cuda_paths:
        environ_cp['TF_CUDA_PATHS'] = tf_cuda_paths


def set_tf_cuda_version(environ_cp):
    """Set TF_CUDA_VERSION."""
    ask_cuda_version = (
        'Please specify the CUDA SDK version you want to use. '
        '[Leave empty to default to CUDA %s]: ') % _DEFAULT_CUDA_VERSION
    tf_cuda_version = get_from_env_or_user_or_default(
        environ_cp, 'TF_CUDA_VERSION', ask_cuda_version, _DEFAULT_CUDA_VERSION)
    environ_cp['TF_CUDA_VERSION'] = tf_cuda_version


def set_tf_cudnn_version(environ_cp):
    """Set TF_CUDNN_VERSION."""
    ask_cudnn_version = (
        'Please specify the cuDNN version you want to use. '
        '[Leave empty to default to cuDNN %s]: ') % _DEFAULT_CUDNN_VERSION
    tf_cudnn_version = get_from_env_or_user_or_default(
        environ_cp, 'TF_CUDNN_VERSION', ask_cudnn_version,
        _DEFAULT_CUDNN_VERSION)
    environ_cp['TF_CUDNN_VERSION'] = tf_cudnn_version


def is_cuda_compatible(lib, cuda_ver, cudnn_ver):
    """Check compatibility between given library and cudnn/cudart libraries."""
    ldd_bin = which('ldd') or '/usr/bin/ldd'
    ldd_out = run_shell([ldd_bin, lib], True)
    ldd_out = ldd_out.split(os.linesep)
    cudnn_pattern = re.compile('.*libcudnn.so\\.?(.*) =>.*$')
    cuda_pattern = re.compile('.*libcudart.so\\.?(.*) =>.*$')
    cudnn = None
    cudart = None
    cudnn_ok = True  # assume no cudnn dependency by default
    cuda_ok = True  # assume no cuda dependency by default
    for line in ldd_out:
        if 'libcudnn.so' in line:
            cudnn = cudnn_pattern.search(line)
            cudnn_ok = False
        elif 'libcudart.so' in line:
            cudart = cuda_pattern.search(line)
            cuda_ok = False
    if cudnn and len(cudnn.group(1)):
        cudnn = convert_version_to_int(cudnn.group(1))
    if cudart and len(cudart.group(1)):
        cudart = convert_version_to_int(cudart.group(1))
    if cudnn is not None:
        cudnn_ok = (cudnn == cudnn_ver)
    if cudart is not None:
        cuda_ok = (cudart == cuda_ver)
    return cudnn_ok and cuda_ok

def set_tf_tensorrt_version(environ_cp):
    """Set TF_TENSORRT_VERSION."""
    if not int(environ_cp.get('TF_NEED_TENSORRT', False)):
        return

    ask_tensorrt_version = (
        'Please specify the TensorRT version you want to use. '
        '[Leave empty to default to TensorRT %s]: '
    ) % _DEFAULT_TENSORRT_VERSION
    tf_tensorrt_version = get_from_env_or_user_or_default(
        environ_cp, 'TF_TENSORRT_VERSION', ask_tensorrt_version,
        _DEFAULT_TENSORRT_VERSION)
    environ_cp['TF_TENSORRT_VERSION'] = tf_tensorrt_version

def set_tf_nccl_version(environ_cp):
    """Set TF_NCCL_VERSION."""
    if 'TF_NCCL_VERSION' in environ_cp:
        return

    ask_nccl_version = (
        'Please specify the locally installed NCCL version you want to use. '
        '[Leave empty to use http://github.com/nvidia/nccl]: ')
    tf_nccl_version = get_from_env_or_user_or_default(
        environ_cp, 'TF_NCCL_VERSION', ask_nccl_version, '')
    environ_cp['TF_NCCL_VERSION'] = tf_nccl_version


def get_native_cuda_compute_capabilities(environ_cp):
    """Get native cuda compute capabilities.

  Args:
    environ_cp: copy of the os.environ.

  Returns:
    string of native cuda compute capabilities, separated by comma.
  """
    device_query_bin = os.path.join(
        environ_cp.get('CUDA_TOOLKIT_PATH'), 'extras/demo_suite/deviceQuery')
    if os.path.isfile(device_query_bin) and os.access(device_query_bin,
                                                      os.X_OK):
        try:
            output = run_shell(device_query_bin).split('\n')
            pattern = re.compile('[0-9]*\\.[0-9]*')
            output = [pattern.search(x) for x in output if 'Capability' in x]
            output = ','.join(x.group() for x in output if x is not None)
        except subprocess.CalledProcessError:
            output = ''
    else:
        output = ''
    return output


def set_tf_cuda_compute_capabilities(environ_cp):
    """Set TF_CUDA_COMPUTE_CAPABILITIES."""
    while True:
        native_cuda_compute_capabilities = get_native_cuda_compute_capabilities(
            environ_cp)
        if not native_cuda_compute_capabilities:
            default_cuda_compute_capabilities = _DEFAULT_CUDA_COMPUTE_CAPABILITIES
        else:
            default_cuda_compute_capabilities = native_cuda_compute_capabilities

        ask_cuda_compute_capabilities = (
            'Please specify a list of comma-separated '
            'CUDA compute capabilities you want to '
            'build with.\nYou can find the compute '
            'capability of your device at: '
            'https://developer.nvidia.com/cuda-gpus.\nPlease'
            ' note that each additional compute '
            'capability significantly increases your '
            'build time and binary size, and that '
            'we only supports compute '
            'capabilities >= 5.0 [Default is: %s]: ' %
            default_cuda_compute_capabilities)
        tf_cuda_compute_capabilities = get_from_env_or_user_or_default(
            environ_cp, 'TF_CUDA_COMPUTE_CAPABILITIES',
            ask_cuda_compute_capabilities, default_cuda_compute_capabilities)
        # Check whether all capabilities from the input is valid
        all_valid = True
        # Remove all whitespace characters before splitting the string
        # that users may insert by accident, as this will result in error
        tf_cuda_compute_capabilities = ''.join(
            tf_cuda_compute_capabilities.split())
        for compute_capability in tf_cuda_compute_capabilities.split(','):
            m = re.match('[0-9]+.[0-9]+', compute_capability)
            if not m:
                print('Invalid compute capability: %s' % compute_capability)
                all_valid = False
            else:
                ver = float(m.group(0))
                if ver < 5.0:
                    print(
                        'ERROR: We only supports CUDA compute capabilities 5.0 '
                        'and higher. Please re-specify the list of compute '
                        'capabilities excluding version %s.' % ver)
                    all_valid = False
                if ver < 3.5:
                    print(
                        'WARNING: XLA does not support CUDA compute capabilities '
                        'lower than 3.5. Disable XLA when running on older GPUs.'
                    )

        if all_valid:
            break

        # Reset and Retry
        environ_cp['TF_CUDA_COMPUTE_CAPABILITIES'] = ''

    # Set TF_CUDA_COMPUTE_CAPABILITIES
    environ_cp['TF_CUDA_COMPUTE_CAPABILITIES'] = tf_cuda_compute_capabilities
    write_action_env_to_bazelrc('TF_CUDA_COMPUTE_CAPABILITIES',
                                tf_cuda_compute_capabilities)


def set_other_cuda_vars(environ_cp):
    """Set other CUDA related variables."""
    # If CUDA is enabled, always use GPU during build and test.
    if environ_cp.get('TF_CUDA_CLANG') == '1':
        write_to_bazelrc('build --config=cuda_clang')
    else:
        write_to_bazelrc('build --config=cuda')


def set_host_cxx_compiler(environ_cp):
    """Set HOST_CXX_COMPILER."""
    default_cxx_host_compiler = which('g++') or ''

    host_cxx_compiler = prompt_loop_or_load_from_env(
        environ_cp,
        var_name='HOST_CXX_COMPILER',
        var_default=default_cxx_host_compiler,
        ask_for_var=('Please specify which C++ compiler should be used as the '
                     'host C++ compiler.'),
        check_success=os.path.exists,
        error_msg='Invalid C++ compiler path. %s cannot be found.',
    )

    write_action_env_to_bazelrc('HOST_CXX_COMPILER', host_cxx_compiler)


def set_host_c_compiler(environ_cp):
    """Set HOST_C_COMPILER."""
    default_c_host_compiler = which('gcc') or ''

    host_c_compiler = prompt_loop_or_load_from_env(
        environ_cp,
        var_name='HOST_C_COMPILER',
        var_default=default_c_host_compiler,
        ask_for_var=(
            'Please specify which C compiler should be used as the host '
            'C compiler.'),
        check_success=os.path.exists,
        error_msg='Invalid C compiler path. %s cannot be found.',
    )

    write_action_env_to_bazelrc('HOST_C_COMPILER', host_c_compiler)


def system_specific_test_config(environ_cp):
    """Add default build and test flags required for TF tests to bazelrc."""
    write_to_bazelrc('test --flaky_test_attempts=3')
    write_to_bazelrc('test --test_size_filters=small,medium')

    # Each instance of --test_tag_filters or --build_tag_filters overrides all
    # previous instances, so we need to build up a complete list and write a
    # single list of filters for the .bazelrc file.

    if environ_cp.get('TF_NEED_CUDA', None) == '1':
        write_to_bazelrc('test --test_env=LD_LIBRARY_PATH')

def set_system_libs_flag(environ_cp):
    syslibs = environ_cp.get('TF_SYSTEM_LIBS', '')
    if syslibs:
        if ',' in syslibs:
            syslibs = ','.join(sorted(syslibs.split(',')))
        else:
            syslibs = ','.join(sorted(syslibs.split()))
        write_action_env_to_bazelrc('TF_SYSTEM_LIBS', syslibs)

    if 'PREFIX' in environ_cp:
        write_to_bazelrc('build --define=PREFIX=%s' % environ_cp['PREFIX'])
    if 'LIBDIR' in environ_cp:
        write_to_bazelrc('build --define=LIBDIR=%s' % environ_cp['LIBDIR'])
    if 'INCLUDEDIR' in environ_cp:
        write_to_bazelrc(
            'build --define=INCLUDEDIR=%s' % environ_cp['INCLUDEDIR'])

def config_info_line(name, help_text):
    """Helper function to print formatted help text for Bazel config options."""
    print('\t--config=%-12s\t# %s' % (name, help_text))


def validate_cuda_config(environ_cp):
    """Run find_cuda_config.py and return cuda_toolkit_path, or None."""

    def maybe_encode_env(env):
        """Encodes unicode in env to str on Windows python 2.x."""
        if sys.version_info[0] != 2:
            return env
        for k, v in env.items():
            if isinstance(k, unicode):
                k = k.encode('ascii')
            if isinstance(v, unicode):
                v = v.encode('ascii')
            env[k] = v
        return env

    cuda_libraries = ['cuda', 'cudnn']
    if int(environ_cp.get('TF_NEED_TENSORRT', False)):
        cuda_libraries.append('tensorrt')
    if environ_cp.get('TF_NCCL_VERSION', None):
        cuda_libraries.append('nccl')

    proc = subprocess.Popen(
        [environ_cp['PYTHON_BIN_PATH'], 'tools/gpus/find_cuda_config.py'] +
        cuda_libraries,
        stdout=subprocess.PIPE,
        env=maybe_encode_env(environ_cp))

    if proc.wait():
        # Errors from find_cuda_config.py were sent to stderr.
        print('Asking for detailed CUDA configuration...\n')
        return False

    config = dict(
        tuple(line.decode('ascii').rstrip().split(': '))
        for line in proc.stdout)

    print('Found CUDA %s in:' % config['cuda_version'])
    print('    %s' % config['cuda_library_dir'])
    print('    %s' % config['cuda_include_dir'])

    print('Found cuDNN %s in:' % config['cudnn_version'])
    print('    %s' % config['cudnn_library_dir'])
    print('    %s' % config['cudnn_include_dir'])

    if 'tensorrt_version' in config:
        print('Found TensorRT %s in:' % config['tensorrt_version'])
        print('    %s' % config['tensorrt_library_dir'])
        print('    %s' % config['tensorrt_include_dir'])

    if config.get('nccl_version', None):
        print('Found NCCL %s in:' % config['nccl_version'])
        print('    %s' % config['nccl_library_dir'])
        print('    %s' % config['nccl_include_dir'])

    print('\n')

    environ_cp['CUDA_TOOLKIT_PATH'] = config['cuda_toolkit_path']
    return True

def set_overall_build_config():
    overall_text = """
## The following was adapted from tensorflow/.bazelrc

# This config refers to building with CUDA available. It does not necessarily
# mean that we build CUDA op kernels.
build:using_cuda --define=using_cuda=true
build:using_cuda --action_env TF_NEED_CUDA=1
build:using_cuda --crosstool_top=@local_config_cuda//crosstool:toolchain

# This config refers to building CUDA op kernels with nvcc.
build:cuda --config=using_cuda
build:cuda --define=using_cuda_nvcc=true

# This config refers to building CUDA op kernels with clang.
build:cuda_clang --config=using_cuda
build:cuda_clang --define=using_cuda_clang=true
build:cuda_clang --define=using_clang=true
build:cuda_clang --action_env TF_CUDA_CLANG=1

build:tensorrt --action_env TF_NEED_TENSORRT=1
build:nonccl --define=no_nccl_support=true
"""
    with open(_TF_BAZELRC, 'a') as f:
        f.write(overall_text)

def default_workspace_directory():
    current_dir = os.path.dirname(__file__)
    if len(current_dir) == 0:
        current_dir = "."
    return os.path.abspath(current_dir + "/..")

def main():
    if not is_linux():
        raise ValueError("Currently ONLY Linux platform is supported.")

    global _TF_WORKSPACE_ROOT
    global _TF_BAZELRC
    global _TF_CURRENT_BAZEL_VERSION

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--workspace',
        type=str,
        default=default_workspace_directory(),
        help='The absolute path to your active Bazel workspace.')
    parser.add_argument(
        '--output_file',
        type=str,
        default='.apollo.bazelrc',
        help='Path of the bazelrc file to write to (relative to workspace directory)')

    args = parser.parse_args()

    _TF_WORKSPACE_ROOT = args.workspace
    _TF_BAZELRC = os.path.join(_TF_WORKSPACE_ROOT, args.output_file)

    # Make a copy of os.environ to be clear when functions and getting and setting
    # environment variables.
    environ_cp = dict(os.environ)

    try:
        current_bazel_version = check_bazel_version(_TF_MIN_BAZEL_VERSION,
                                                    _TF_MAX_BAZEL_VERSION)
    except subprocess.CalledProcessError as e:
        print("Error checking bazel version: ",
              e.output.decode('UTF-8').strip())
        raise e

    _TF_CURRENT_BAZEL_VERSION = convert_version_to_int(current_bazel_version)

    reset_tf_configure_bazelrc()
    setup_python(environ_cp)

    environ_cp['TF_NEED_CUDA'] = str(
        int(get_var(environ_cp, 'TF_NEED_CUDA', 'CUDA', False)))
    if environ_cp.get('TF_NEED_CUDA') == '1':
        set_action_env_var(
            environ_cp,
            'TF_NEED_TENSORRT',
            'TensorRT',
            False,
            bazel_config_name='tensorrt')

        environ_save = dict(environ_cp)
        for _ in range(_DEFAULT_PROMPT_ASK_ATTEMPTS):

            if validate_cuda_config(environ_cp):
                cuda_env_names = [
                    'TF_CUDA_VERSION',
                    'TF_CUBLAS_VERSION',
                    'TF_CUDNN_VERSION',
                    'TF_TENSORRT_VERSION',
                    'TF_NCCL_VERSION',
                    'TF_CUDA_PATHS',
                    # Items below are for backwards compatibility when not using
                    # TF_CUDA_PATHS.
                    'CUDA_TOOLKIT_PATH',
                    'CUDNN_INSTALL_PATH',
                    'NCCL_INSTALL_PATH',
                    'NCCL_HDR_PATH',
                    'TENSORRT_INSTALL_PATH'
                ]
                # Note: set_action_env_var above already writes to bazelrc.
                for name in cuda_env_names:
                    if name in environ_cp:
                        write_action_env_to_bazelrc(name, environ_cp[name])
                break

            # Restore settings changed below if CUDA config could not be validated.
            environ_cp = dict(environ_save)

            set_tf_cuda_version(environ_cp)
            set_tf_cudnn_version(environ_cp)
            set_tf_tensorrt_version(environ_cp)
            set_tf_nccl_version(environ_cp)

            set_tf_cuda_paths(environ_cp)

        else:
            raise UserInputError(
                'Invalid CUDA setting were provided %d '
                'times in a row. Assuming to be a scripting mistake.' %
                _DEFAULT_PROMPT_ASK_ATTEMPTS)

        set_tf_cuda_compute_capabilities(environ_cp)
        if 'LD_LIBRARY_PATH' in environ_cp and environ_cp.get(
                'LD_LIBRARY_PATH') != '1':
            write_action_env_to_bazelrc('LD_LIBRARY_PATH',
                                        environ_cp.get('LD_LIBRARY_PATH'))

        set_tf_cuda_clang(environ_cp)
        if environ_cp.get('TF_CUDA_CLANG') == '1':
            # Set up which clang we should use as the cuda / host compiler.
            set_clang_cuda_compiler_path(environ_cp)
        else:
            # Set up which gcc nvcc should use as the host compiler
            set_gcc_host_compiler_path(environ_cp)
        set_other_cuda_vars(environ_cp)
    else:
        # CUDA not required. Ask whether we should download the clang toolchain and
        # use it for the CPU build.
        pass
        # set_tf_download_clang(environ_cp)

#    set_system_libs_flag(environ_cp)
#    set_cc_opt_flags(environ_cp)
#    system_specific_test_config(environ_cp)
    set_overall_build_config()

#    print('Preconfigured Bazel build configs. You can use any of the below by '
#          'adding "--config=<>" to your build command. See .bazelrc for more '
#          'details.')
#    config_info_line('mkl', 'Build with MKL support.')
#    config_info_line('ngraph', 'Build with Intel nGraph support.')
#    config_info_line('numa', 'Build with NUMA support.')
#    config_info_line(
#        'dynamic_kernels',
#        '(Experimental) Build kernels into separate shared objects.')
    print('Preconfigured Bazel build configs to DISABLE default on features:')
    config_info_line('nonccl', 'Disable NVIDIA NCCL support.')
#    config_info_line('noaws', 'Disable AWS S3 filesystem support.')
#    config_info_line('nogcp', 'Disable GCP support.')
#    config_info_line('nohdfs', 'Disable HDFS support.')

if __name__ == '__main__':
    main()
