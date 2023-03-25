#! /usr/bin/env python3

# ****************************************************************************
# Copyright 2020 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Copyright 2017 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# ==============================================================================

# -*- coding: utf-8 -*-
"""Script to get build parameters interactively from user.
Adapted significantly from tensorflow.git/configure.py .
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import os
import platform
import re
import subprocess
import sys

from distutils.util import (strtobool)

# pylint: disable=g-import-not-at-top
try:
    from shutil import which
except ImportError:
    from distutils.spawn import find_executable as which
# pylint: enable=g-import-not-at-top

_DEFAULT_CUDA_VERSION = '10'
_DEFAULT_CUDNN_VERSION = '7'
_DEFAULT_TENSORRT_VERSION = '7'
_DEFAULT_MIGRAPHX_VERSION = '2'
_DEFAULT_CUDA_COMPUTE_CAPABILITIES = '3.7,5.2,6.0,6.1,7.0,7.2,7.5'
_DEFAULT_ROCM_COMPUTE_CAPABILITIES = 'gfx900,gfx906'
_DEFAULT_PYTHON_LIB_PATH = '/usr/lib/python3/dist-packages'

_DEFAULT_PROMPT_ASK_ATTEMPTS = 3

_APOLLO_ROOT_DIR = ''
_APOLLO_BAZELRC = '.apollo.bazelrc'

_APOLLO_CURRENT_BAZEL_VERSION = None
_APOLLO_MIN_BAZEL_VERSION = '2.0.0'
_APOLLO_INSIDE_DOCKER = True
_APOLLO_DOCKER_STAGE = "dev"

_INTERACTIVE_MODE = True

class color:
    GREEN = '\033[32m'
    RED = '\033[0;31m'
    BLUE = '\033[1;34;48m'
    YELLOW = '\033[33m'
    NO_COLOR = '\033[0m'

_INFO = '[' + color.BLUE + 'INFO' + color.NO_COLOR + '] '
_WARNING = color.YELLOW + '[WARNING] ' + color.NO_COLOR
_ERROR = '[' + color.RED + 'ERROR' + color.NO_COLOR + '] '

class UserInputError(Exception):
    pass


def is_linux():
    return platform.system() == 'Linux'


def inside_docker():
    return os.path.isfile('/.dockerenv')


def docker_stage():
    default_apollo_stage = "dev"
    if not inside_docker():
        return default_apollo_stage
    stage_conf = "/etc/apollo.conf"
    if not os.path.exists(stage_conf) or not os.path.isfile(stage_conf):
        return default_apollo_stage

    with open("/etc/apollo.conf") as f:
        for line in f:
            line = line.strip()
            if line.startswith("stage="):
                return line.split("=")[-1]
    return default_apollo_stage


def default_root_dir():
    current_dir = os.path.dirname(__file__)
    if len(current_dir) == 0:
        current_dir = '.'
    return os.path.abspath(os.path.join(current_dir, '..'))


def get_input(question):
    try:
        try:
            answer = raw_input(question)
        except NameError:
            answer = input(question)  # pylint: disable=bad-builtin
    except EOFError:
        answer = ''
    return answer


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def write_to_bazelrc(line):
    with open(_APOLLO_BAZELRC, 'a') as f:
        f.write(line + '\n')


def write_blank_line_to_bazelrc():
    with open(_APOLLO_BAZELRC, 'a') as f:
        f.write('\n')


def write_action_env_to_bazelrc(var_name, var):
    write_to_bazelrc('build --action_env {}="{}"'.format(var_name, str(var)))


def write_build_var_to_bazelrc(bazel_config_name, option_name):
    # TODO(build): Migrate all users of configure.py to use --config Bazel
    # options and not to set build configs through environment variables.
    write_to_bazelrc('build:%s --define %s=true' % (bazel_config_name,
                                                    option_name))


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
    return output.decode("UTF-8").strip()


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
        if os.path.isdir(path) and not path.startswith(
                os.path.join(_APOLLO_ROOT_DIR, "bazel-bin")):
            paths.append(path)
    return paths


def get_python_major_version(python_bin_path):
    """Get the python major version."""
    return run_shell(
        [python_bin_path, '-c', 'import sys; print(sys.version[0])'])


def setup_common_dirs(environ_cp):
    """Setup --distdir and --output_user_root directories"""
    cache_dir = os.path.join(_APOLLO_ROOT_DIR, '.cache')
    dist_dir = os.path.join(_APOLLO_ROOT_DIR, '.cache/distdir')

    # if cyber/setup.bash not sourced
    if 'APOLLO_CACHE_DIR' in environ_cp:
        cache_dir = environ_cp['APOLLO_CACHE_DIR']

    if 'APOLLO_BAZEL_DIST_DIR' in environ_cp:
        dist_dir = environ_cp['APOLLO_BAZEL_DIST_DIR']

    write_to_bazelrc('startup --output_user_root="{}/bazel"'.format(cache_dir))
    write_to_bazelrc('common --distdir="{}"'.format(dist_dir))
    write_to_bazelrc('common --repository_cache="{}/repos"'.format(cache_dir))
    write_to_bazelrc('build --disk_cache="{}/build"'.format(cache_dir))
    write_to_bazelrc('')


def setup_python(environ_cp):
    """Setup python related env variables."""
    if not _INTERACTIVE_MODE:
        setup_python_non_interactively(environ_cp)
    else:
        setup_python_interactively(environ_cp)


def setup_python_non_interactively(environ_cp):
    """Setup python related env variables non-interactively."""
    # Get PYTHON_BIN_PATH, default is the current running python.
    python_bin_path = sys.executable
    if not os.path.exists(python_bin_path):
        print((_ERROR + 'Invalid python path: {} cannot be found.').format(
            python_bin_path))
        sys.exit(1)
    if not os.path.isfile(python_bin_path) or not os.access(
            python_bin_path, os.X_OK):
        print((_ERROR + '{} is not executable.').format(python_bin_path))
        sys.exit(1)
    python_major_version = get_python_major_version(python_bin_path)
    if python_major_version != '3':
        print(_ERROR + 'Python 2 was retired on April 2020. Use Python 3 instead.')
        sys.exit(1)

    environ_cp['PYTHON_BIN_PATH'] = python_bin_path
    # Get PYTHON_LIB_PATH
    python_lib_path = environ_cp.get('PYTHON_LIB_PATH')
    if not python_lib_path:
        python_lib_paths = get_python_path(environ_cp, python_bin_path)
        print(_INFO + ('Found possible ' + color.GREEN + 'Python' + color.NO_COLOR + ' library paths:\n       %s') %
              '\n       '.join(python_lib_paths))
        if _DEFAULT_PYTHON_LIB_PATH in python_lib_paths:
            default_python_lib_path = _DEFAULT_PYTHON_LIB_PATH
        else:
            default_python_lib_path = python_lib_paths[0]
        python_lib_path = default_python_lib_path

    # Set-up env variables used by python_configure.bzl
    write_action_env_to_bazelrc('PYTHON_BIN_PATH', python_bin_path)
    write_action_env_to_bazelrc('PYTHON_LIB_PATH', python_lib_path)
    write_to_bazelrc('build --python_path=\"{}\"'.format(python_bin_path))

    # If choosen python_lib_path is from a path specified in the PYTHONPATH
    # variable, need to tell bazel to include PYTHONPATH
    if environ_cp.get('PYTHONPATH'):
        python_paths = environ_cp.get('PYTHONPATH').split(':')
        if python_lib_path in python_paths:
            write_action_env_to_bazelrc('PYTHONPATH',
                                        environ_cp.get('PYTHONPATH'))


def setup_python_interactively(environ_cp):
    """Setup python related env variables interactively."""
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
            print((_ERROR + 'Invalid python path: {} cannot be found.').format(
                python_bin_path))
        else:
            print((_ERROR + '{} is not executable.  Is it the python binary?').format(
                python_bin_path))
        environ_cp['PYTHON_BIN_PATH'] = ''

    python_major_version = get_python_major_version(python_bin_path)
    if python_major_version != '3':
        print(_ERROR + 'Python 2 was Retired on April 2020. Use Python 3 instead.')
        sys.exit(1)

    # Get PYTHON_LIB_PATH
    python_lib_path = environ_cp.get('PYTHON_LIB_PATH')
    if not python_lib_path:
        python_lib_paths = get_python_path(environ_cp, python_bin_path)
        print(_INFO + ('Found possible Python library paths:\n       %s') %
              '\n       '.join(python_lib_paths))
        default_python_lib_path = python_lib_paths[0]
        python_lib_path = get_input(
            'Please input the desired Python library path to use.  '
            'Default is [{}]\n'.format(python_lib_paths[0]))
        if not python_lib_path:
            python_lib_path = default_python_lib_path
        environ_cp['PYTHON_LIB_PATH'] = python_lib_path

    # Set-up env variables used by python_configure.bzl
    write_action_env_to_bazelrc('PYTHON_BIN_PATH', python_bin_path)
    write_action_env_to_bazelrc('PYTHON_LIB_PATH', python_lib_path)
    write_to_bazelrc('build --python_path=\"{}\"'.format(python_bin_path))
    environ_cp['PYTHON_BIN_PATH'] = python_bin_path

    # If choosen python_lib_path is from a path specified in the PYTHONPATH
    # variable, need to tell bazel to include PYTHONPATH
    if environ_cp.get('PYTHONPATH'):
        python_paths = environ_cp.get('PYTHONPATH').split(':')
        if python_lib_path in python_paths:
            write_action_env_to_bazelrc('PYTHONPATH',
                                        environ_cp.get('PYTHONPATH'))


def reset_apollo_bazelrc():
    """Reset file that contains customized config settings."""
    open(_APOLLO_BAZELRC, 'w').close()


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
            print((_ERROR + 'Invalid selection: {}').format(user_input_origin))
    return var


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


def check_bazel_version(min_version):
    """Check installed bazel version.

    Args:
        min_version: string for minimum bazel version (must exist!).

    Returns:
        The bazel version detected.
    """
    if which('bazel') is None:
        print(_ERROR + 'Cannot find bazel. Please install bazel first.')
        sys.exit(0)

    stderr = open(os.devnull, 'wb')
    curr_version = run_shell(
        ['bazel', '--version'], allow_non_zero=True, stderr=stderr)
    if curr_version.startswith('bazel '):
        curr_version = curr_version.split('bazel ')[1]

    min_version_int = convert_version_to_int(min_version)
    curr_version_int = convert_version_to_int(curr_version)

    # Check if current bazel version can be detected properly.
    if not curr_version_int:
        print(_WARNING + 'Current bazel installation is not a release version.')
        print('          Make sure you are running at least bazel %s' % min_version)
        return curr_version

    print((_INFO + 'You have ' + color.GREEN + 'bazel %s' + color.NO_COLOR + ' installed.') % curr_version)

    if curr_version_int < min_version_int:
        print((_ERROR + 'Please upgrade your bazel installation to version %s or higher') % min_version)
        sys.exit(1)
    return curr_version


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

    if not _INTERACTIVE_MODE:
        gcc_host_compiler_path = default_gcc_host_compiler_path
        if os.path.islink(gcc_host_compiler_path):
            gcc_host_compiler_path = os.path.realpath(gcc_host_compiler_path)
    else:
        gcc_host_compiler_path = prompt_loop_or_load_from_env(
            environ_cp,
            var_name='GCC_HOST_COMPILER_PATH',
            var_default=default_gcc_host_compiler_path,
            ask_for_var='Please specify which gcc should be used by nvcc as the host compiler.',
            check_success=os.path.exists,
            resolve_symlinks=True,
            error_msg=_ERROR + 'Invalid gcc path. %s cannot be found.',
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


def set_cuda_paths(environ_cp):
    """Set TF_CUDA_PATHS."""
    ask_cuda_paths = (
        'Please specify the comma-separated list of base paths to look for CUDA '
        'libraries and headers. [Leave empty to use the default]: ')
    tf_cuda_paths = get_from_env_or_user_or_default(
        environ_cp, 'TF_CUDA_PATHS', ask_cuda_paths, '')
    if tf_cuda_paths:
        environ_cp['TF_CUDA_PATHS'] = tf_cuda_paths


def set_cuda_version(environ_cp):
    """Set TF_CUDA_VERSION."""
    ask_cuda_version = (
        'Please specify the CUDA SDK version you want to use. '
        '[Leave empty to default to CUDA %s]: ') % _DEFAULT_CUDA_VERSION
    tf_cuda_version = get_from_env_or_user_or_default(
        environ_cp, 'TF_CUDA_VERSION', ask_cuda_version, _DEFAULT_CUDA_VERSION)
    environ_cp['TF_CUDA_VERSION'] = tf_cuda_version


def set_cudnn_version(environ_cp):
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


def set_tensorrt_version(environ_cp):
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


def set_migraphx_version(environ_cp):
    """Set TF_MIGRAPHX_VERSION."""
    if not int(environ_cp.get('TF_NEED_MIGRAPHX', False)):
        return

    ask_migraphx_version = (
        'Please specify the MIGraphX version you want to use. '
        '[Leave empty to default to MIGraphX %s]: '
    ) % _DEFAULT_MIGRAPHX_VERSION
    tf_migraphx_version = get_from_env_or_user_or_default(
        environ_cp, 'TF_MIGRAPHX_VERSION', ask_migraphx_version,
        _DEFAULT_MIGRAPHX_VERSION)
    environ_cp['TF_MIGRAPHX_VERSION'] = tf_migraphx_version


def set_nccl_version(environ_cp):
    """Set TF_NCCL_VERSION."""
    if 'TF_NCCL_VERSION' in environ_cp:
        return

    ask_nccl_version = (
        'Please specify the locally installed NCCL version you want to use. '
    )
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


def get_native_rocm_compute_capabilities(environ_cp):
    """Get native rocm compute capabilities.

    Args:
      environ_cp: copy of the os.environ.

    Returns:
      string of native rocm compute capabilities , separated by comma.
      (e.g. "gfx000,gfx906")
    """
    device_query_bin = os.path.join(
        environ_cp.get('ROCM_TOOLKIT_PATH'), 'bin/rocm_agent_enumerator')
    if os.path.isfile(device_query_bin) and os.access(device_query_bin,
                                                      os.X_OK):
        try:
            output = run_shell(device_query_bin).split('\n')

            output = ','.join(output)
        except subprocess.CalledProcessError:
            output = ''
    else:
        output = ''
    return output


def set_cuda_compute_capabilities(environ_cp):
    """Set TF_CUDA_COMPUTE_CAPABILITIES."""
    if not _INTERACTIVE_MODE:
        native_cuda_compute_capabilities = get_native_cuda_compute_capabilities(
            environ_cp)
        if native_cuda_compute_capabilities:
            tf_cuda_compute_capabilities = native_cuda_compute_capabilities
        else:
            tf_cuda_compute_capabilities = _DEFAULT_CUDA_COMPUTE_CAPABILITIES
        # Set TF_CUDA_COMPUTE_CAPABILITIES
        environ_cp['TF_CUDA_COMPUTE_CAPABILITIES'] = tf_cuda_compute_capabilities
        write_action_env_to_bazelrc('TF_CUDA_COMPUTE_CAPABILITIES',
                                    tf_cuda_compute_capabilities)
        return

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
            'capabilities >= 3.7 [Default is: %s]: ' %
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
                print((_ERROR + 'Invalid compute capability: %s') % compute_capability)
                all_valid = False
            else:
                ver = float(m.group(0))
                if ver < 3.5:
                    print((_ERROR + 'We only supports CUDA compute capabilities 3.7 and higher.'
                        '        Please re-specify the list of compute capabilities excluding version %s.') % ver)
                    all_valid = False

        if all_valid:
            break

        # Reset and Retry
        environ_cp['TF_CUDA_COMPUTE_CAPABILITIES'] = ''

    # Set TF_CUDA_COMPUTE_CAPABILITIES
    environ_cp['TF_CUDA_COMPUTE_CAPABILITIES'] = tf_cuda_compute_capabilities
    write_action_env_to_bazelrc('TF_CUDA_COMPUTE_CAPABILITIES',
                                tf_cuda_compute_capabilities)


def set_rocm_compute_capabilities(environ_cp):
    """Set TF_ROCM_COMPUTE_CAPABILITIES."""
    native_rocm_compute_capabilities = get_native_rocm_compute_capabilities(
        environ_cp)
    if native_rocm_compute_capabilities:
        tf_rocm_compute_capabilities = native_rocm_compute_capabilities
    else:
        tf_rocm_compute_capabilities = _DEFAULT_ROCM_COMPUTE_CAPABILITIES
    # Set TF_ROCM_COMPUTE_CAPABILITIES
    environ_cp['TF_ROCM_COMPUTE_CAPABILITIES'] = tf_rocm_compute_capabilities
    write_action_env_to_bazelrc('TF_ROCM_COMPUTE_CAPABILITIES',
                                tf_rocm_compute_capabilities)
    return


def set_other_cuda_vars(environ_cp):
    """Set other CUDA related variables."""
    # If CUDA is enabled, always use GPU during build and test.
    # write_to_bazelrc('build --config=cuda')
    pass


def validate_cuda_config(environ_cp):
    """Run find_cuda_config.py and return cuda_toolkit_path, or None."""

    cuda_libraries = ['cuda', 'cudnn']
    if int(environ_cp.get('TF_NEED_TENSORRT', False)):
        cuda_libraries.append('tensorrt')
    if environ_cp.get('TF_NCCL_VERSION', None):
        cuda_libraries.append('nccl')

    find_cuda_script = os.path.join(
        _APOLLO_ROOT_DIR,
        'third_party/gpus/find_cuda_config.py')
    proc = subprocess.Popen(
        [environ_cp['PYTHON_BIN_PATH'], find_cuda_script]
        + cuda_libraries,
        stdout=subprocess.PIPE,
        env=environ_cp)

    if proc.wait():
        # Errors from find_cuda_config.py were sent to stderr.
        print(_INFO + 'Asking for detailed CUDA configuration...\n')
        return False

    config = dict(
        tuple(line.decode('ascii').rstrip().split(': '))
        for line in proc.stdout)

    print((_INFO + 'Found ' + color.GREEN + 'CUDA %s' + color.NO_COLOR + ' in:') % config['cuda_version'])
    print('       %s' % config['cuda_library_dir'])
    print('       %s' % config['cuda_include_dir'])

    print((_INFO + 'Found ' + color.GREEN + 'cuDNN %s' + color.NO_COLOR + ' in:') % config['cudnn_version'])
    print('       %s' % config['cudnn_library_dir'])
    print('       %s' % config['cudnn_include_dir'])

    if 'tensorrt_version' in config:
        print((_INFO + 'Found ' + color.GREEN + 'TensorRT %s' + color.NO_COLOR + ' in:') % config['tensorrt_version'])
        print('       %s' % config['tensorrt_library_dir'])
        print('       %s' % config['tensorrt_include_dir'])

    if config.get('nccl_version', None):
        print((_INFO + 'Found ' + color.GREEN + 'NCCL %s' + color.NO_COLOR + ' in:') % config['nccl_version'])
        print('       %s' % config['nccl_library_dir'])
        print('       %s' % config['nccl_include_dir'])

    environ_cp['CUDA_TOOLKIT_PATH'] = config['cuda_toolkit_path']
    return True


def validate_rocm_config(environ_cp):
    """Run find_rocm_config.py and return r_toolkit_path, or None."""

    find_rocm_script = os.path.join(
        _APOLLO_ROOT_DIR,
        'third_party/gpus/find_rocm_config.py')
    proc = subprocess.Popen(
        [environ_cp['PYTHON_BIN_PATH'], find_rocm_script],
        stdout=subprocess.PIPE,
        env=environ_cp)

    if proc.wait():
        # Errors from find_rocm_config.py were sent to stderr.
        print(_INFO + 'Asking for detailed ROCM configuration...\n')
        return False

    config = dict(
        tuple(line.decode('ascii').rstrip().split(': '))
        for line in proc.stdout)

    print((_INFO + 'Found ' + color.GREEN + 'ROCm %s' + color.NO_COLOR + ' in:') % config['rocm_version_number'])
    print('       %s' % config['rocm_toolkit_path'])
    print('       %s' % config['rocm_header_path'])

    print((_INFO + 'Found ' + color.GREEN + 'HIP %s' + color.NO_COLOR + ' in:') % config['hipruntime_version_number'])
    print('       %s' % config['hipruntime_library_dir'])
    print('       %s' % config['hipruntime_include_dir'])

    print((_INFO + 'Found ' + color.GREEN + 'hipBLAS %s' + color.NO_COLOR + ' in:') % config['hipblas_version_number'])
    print('       %s' % config['hipblas_library_dir'])
    print('       %s' % config['hipblas_include_dir'])

    print((_INFO + 'Found ' + color.GREEN + 'rocBLAS %s' + color.NO_COLOR + ' in:') % config['rocblas_version_number'])
    print('       %s' % config['rocblas_library_dir'])
    print('       %s' % config['rocblas_include_dir'])

    print((_INFO + 'Found ' + color.GREEN + 'MIOpen %s' + color.NO_COLOR + ' in:') % config['miopen_version_number'])
    print('       %s' % config['miopen_library_dir'])
    print('       %s' % config['miopen_include_dir'])

    print((_INFO + 'Found ' + color.GREEN + 'MIGraphX %s' + color.NO_COLOR + ' in:') % config['migraphx_version_number'])
    print('       %s' % config['migraphx_library_dir'])
    print('       %s' % config['migraphx_include_dir'])

    environ_cp['ROCM_TOOLKIT_PATH'] = config['rocm_toolkit_path']
    return True


def setup_cuda_family_config_interactively(environ_cp):
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
                # Items below are for backwards compatibility
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

        # Restore settings changed below if CUDA config could not be
        # validated.
        environ_cp = dict(environ_save)

        # TODO(build): revisit these settings
        set_cuda_version(environ_cp)
        set_cudnn_version(environ_cp)
        set_tensorrt_version(environ_cp)
        set_nccl_version(environ_cp)
        set_cuda_paths(environ_cp)

    else:
        raise UserInputError(
            'Invalid CUDA setting were provided %d '
            'times in a row. Assuming to be a scripting mistake.' %
            _DEFAULT_PROMPT_ASK_ATTEMPTS)


def setup_cuda_family_config_non_interactively(environ_cp):
    if not validate_cuda_config(environ_cp):
        print(_ERROR + "Cannot validate_cuda_config non-interactively. Aborting ...")
        sys.exit(1)

    cuda_env_names = [
        'TF_CUDA_VERSION',
        'TF_CUBLAS_VERSION',
        'TF_CUDNN_VERSION',
        'TF_TENSORRT_VERSION',
        'TF_NCCL_VERSION',
        'TF_CUDA_PATHS',
        # Items below are for backwards compatibility
        'CUDA_TOOLKIT_PATH',
        'CUDNN_INSTALL_PATH',
        'NCCL_INSTALL_PATH',
        'NCCL_HDR_PATH',
        'TENSORRT_INSTALL_PATH'
    ]
    for name in cuda_env_names:
        if name in environ_cp:
            write_action_env_to_bazelrc(name, environ_cp[name])


def setup_cuda_family_config(environ_cp):
    """Setup CUDA/cuDNN/TensorRT/NCCL action env."""
    if not _INTERACTIVE_MODE:
        setup_cuda_family_config_non_interactively(environ_cp)
    else:
        setup_cuda_family_config_interactively(environ_cp)


def setup_rocm_family_config(environ_cp):
    """Setup ROCm/hip/hipblas/rocblas/miopen action env."""
    if not validate_rocm_config(environ_cp):
        print(_ERROR + "Cannot validate_rocm_config non-interactively. Aborting ...")
        sys.exit(1)

    rocm_env_names = [
        'TF_ROCM_VERSION',
        'TF_HIPBLAS_VERSION',
        'TF_ROCBLAS_VERSION',
        'TF_HIP_VERSION',
        'TF_MIOPEN_VERSION',
        # Items below are for backwards compatibility
        'ROCM_TOOLKIT_PATH',
        'HIP_INSTALL_PATH',
        'HIPBLAS_INSTALL_PATH',
        'ROCBLAS_INSTALL_PATH',
        'MIOPEN_INSTALL_PATH'
        'MIGRAPHX_INSTALL_PATH'
    ]
    for name in rocm_env_names:
        if name in environ_cp:
            write_action_env_to_bazelrc(name, environ_cp[name])


def set_other_build_cuda_config(environ_cp):
    build_text = """
# This config refers to building with CUDA available.
build:using_cuda --define=using_cuda=true
build:using_cuda --action_env TF_NEED_CUDA=1
build:using_cuda --crosstool_top=@local_config_cuda//crosstool:toolchain

# This config refers to building CUDA with nvcc.
build:cuda --config=using_cuda
build:cuda --define=using_cuda_nvcc=true

build:tensorrt --action_env TF_NEED_TENSORRT=1
"""
    with open(_APOLLO_BAZELRC, 'a') as f:
        f.write(build_text)


def set_other_build_rocm_config(environ_cp):
    build_text = """
# This config refers to building with ROCm available.
build:using_rocm --define=using_rocm=true
build:using_rocm --action_env TF_NEED_ROCM=1
build:using_rocm --crosstool_top=@local_config_rocm//crosstool:toolchain

# This config refers to building ROCm with hipcc.
build:rocm --config=using_rocm
build:rocm --define=using_rocm_hipcc=true

build:migraphx --action_env TF_NEED_MIGRAPHX=1
"""
    with open(_APOLLO_BAZELRC, 'a') as f:
        f.write(build_text)


def set_tensorrt_config(environ_cp):
    global _APOLLO_DOCKER_STAGE


def main():
    if not is_linux():
        raise ValueError('Currently, only Linux is support.')

    global _APOLLO_ROOT_DIR
    global _APOLLO_BAZELRC
    global _APOLLO_CURRENT_BAZEL_VERSION
    global _APOLLO_INSIDE_DOCKER
    global _INTERACTIVE_MODE
    global _APOLLO_DOCKER_STAGE

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output_file',
        type=str,
        default='.apollo.bazelrc',
        help='Path of the bazelrc file to write to (relative to APOLLO_ROOT_DIR)')

    parser.add_argument('--interactive', type=str2bool, nargs='?',
                        const=True, default=True,
                        help='Run this script interactively')

    args = parser.parse_args()

    _APOLLO_ROOT_DIR = default_root_dir()
    _APOLLO_BAZELRC = os.path.join(_APOLLO_ROOT_DIR, args.output_file)
    _APOLLO_INSIDE_DOCKER = inside_docker()
    _APOLLO_DOCKER_STAGE = docker_stage()
    _INTERACTIVE_MODE = args.interactive

    # Make a copy of os.environ to be clear when functions and getting and setting
    # environment variables.
    environ_cp = dict(os.environ)

    try:
        current_bazel_version = check_bazel_version(_APOLLO_MIN_BAZEL_VERSION)
    except subprocess.CalledProcessError as e:
        print('Error checking bazel version: ',
              e.output.decode('UTF-8').strip())
        raise e

    _APOLLO_CURRENT_BAZEL_VERSION = convert_version_to_int(
        current_bazel_version)

    reset_apollo_bazelrc()
    setup_common_dirs(environ_cp)
    setup_python(environ_cp)

    if strtobool(environ_cp.get('TF_NEED_CUDA', 'False')):
        write_to_bazelrc('build:gpu --config=cuda')
    if strtobool(environ_cp.get('TF_NEED_ROCM', 'False')):
        write_to_bazelrc('build:gpu --config=rocm')

    if _APOLLO_DOCKER_STAGE == "dev":
        if strtobool(environ_cp.get('TF_NEED_CUDA', 'False')):
            environ_cp['TF_NEED_TENSORRT'] = '1'
            write_to_bazelrc('build:gpu --config=tensorrt')
        if strtobool(environ_cp.get('TF_NEED_ROCM', 'False')):
            environ_cp['TF_NEED_MIGRAPHX'] = '1'
            write_to_bazelrc('build:gpu --config=migraphx')

    write_blank_line_to_bazelrc()
    set_gcc_host_compiler_path(environ_cp)
    write_blank_line_to_bazelrc()

    if strtobool(environ_cp.get('TF_NEED_CUDA', 'False')):
        setup_cuda_family_config(environ_cp)
        set_cuda_compute_capabilities(environ_cp)
        set_other_cuda_vars(environ_cp)
        set_other_build_cuda_config(environ_cp)
    if strtobool(environ_cp.get('TF_NEED_ROCM', 'False')):
        setup_rocm_family_config(environ_cp)
        set_rocm_compute_capabilities(environ_cp)
        set_other_build_rocm_config(environ_cp)

    write_build_var_to_bazelrc('teleop', 'WITH_TELEOP')
    if not _APOLLO_INSIDE_DOCKER and 'LD_LIBRARY_PATH' in environ_cp:
        write_action_env_to_bazelrc('LD_LIBRARY_PATH',
                                    environ_cp.get('LD_LIBRARY_PATH'))


if __name__ == '__main__':
    main()
