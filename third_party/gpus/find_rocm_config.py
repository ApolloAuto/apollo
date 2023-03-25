# Copyright 2020 The TensorFlow Authors. All Rights Reserved.
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
"""Prints ROCm library and header directories and versions found on the system.

The script searches for ROCm library and header files on the system, inspects
them to determine their version and prints the configuration to stdout.
The path to inspect is specified through an environment variable (ROCM_PATH).
If no valid configuration is found, the script prints to stderr and
returns an error code.

The script takes the directory specified by the ROCM_PATH environment variable.
The script looks for headers and library files in a hard-coded set of
subdirectories from base path of the specified directory. If ROCM_PATH is not
specified, then "/opt/rocm" is used as it default value

"""

import io
import os
import re
import sys
import glob


def _header_paths():
    """Returns hard-coded set of relative paths to look for header files."""
    return [
        "",
        "include",
        "include/rocm",
        "include/*-linux-gnu",
        ".info"
    ]


def _library_paths():
    """Returns hard-coded set of relative paths to look for library files."""
    return [
        "",
        "lib64",
        "lib",
        "lib/*-linux-gnu",
        "lib64/stubs"
    ]


class ConfigError(Exception):
    pass


def _matches_version(actual_version, required_version):
    """Checks whether some version meets the requirements.

        All elements of the required_version need to be present in the
        actual_version.

            required_version  actual_version  result
            -----------------------------------------
            1                 1.1             True
            1.2               1               False
            1.2               1.3             False
                              1               True

        Args:
          required_version: The version specified by the user.
          actual_version: The version detected from the CUDA installation.
        Returns: Whether the actual version matches the required one.
    """
    if actual_version is None:
        return False

    # Strip spaces from the versions.
    actual_version = str(actual_version).strip()
    required_version = str(required_version).strip()
    return actual_version.startswith(required_version)


def _cartesian_product(first, second):
    """Returns all path combinations of first and second."""
    return [os.path.join(f, s) for f in first for s in second]


def _not_found_error(base_paths, relative_paths, filepattern):
    base_paths = "".join(["\n        '%s'" %
                          path for path in sorted(base_paths)])
    relative_paths = "".join(["\n        '%s'" %
                              path for path in relative_paths])
    return ConfigError(
        "Could not find any %s in any subdirectory:%s\nof:%s\n" %
        (filepattern, relative_paths, base_paths))


def _find_file(base_paths, relative_paths, filepattern):
    for path in _cartesian_product(base_paths, relative_paths):
        for file in glob.glob(os.path.join(path, filepattern)):
            return file
    raise _not_found_error(base_paths, relative_paths, filepattern)


def _find_library(base_paths, library_name, required_version, library_paths):
    """Returns first valid path to the requested library."""
    filepattern = ".".join(["lib" + library_name, "so"]
                           + required_version.split(".")[:1]) + "*"
    return _find_file(base_paths, library_paths(), filepattern)


def _find_versioned_file(base_paths, relative_paths, filepatterns,
                         required_version, get_version):
    """Returns first valid path to a file that matches the requested version."""
    if type(filepatterns) not in [list, tuple]:
        filepatterns = [filepatterns]
    for path in _cartesian_product(base_paths, relative_paths):
        for filepattern in filepatterns:
            for file in glob.glob(os.path.join(path, filepattern)):
                actual_version = get_version(file)
                if _matches_version(actual_version, required_version):
                    return file, actual_version
    raise _not_found_error(
        base_paths, relative_paths,
        ", ".join(filepatterns) + " matching version '%s'" % required_version)


def _find_header(base_paths, header_name, required_version, get_version, header_paths):
    """Returns first valid path to a header that matches the requested version."""
    return _find_versioned_file(base_paths, header_paths(), header_name,
                                required_version, get_version)


def _get_path_if_link(path):
  if os.path.islink(path):
      return os.path.realpath(path)
  return path


def _get_default_rocm_path():
  return "/opt/rocm"


def _get_rocm_install_path():
  """Determines and returns the ROCm installation path."""
  rocm_install_path = os.environ.get("ROCM_PATH", _get_default_rocm_path())
  return _get_path_if_link(rocm_install_path)


def _get_composite_version_number(major, minor, patch):
  return 10000 * major + 100 * minor + patch


def _get_header_version(path, name):
  """Returns preprocessor defines in C header file."""
  for line in io.open(path, "r", encoding="utf-8"):
    match = re.match(r"#define %s +(\d+)" % name, line)
    if match:
      value = match.group(1)
      return int(value)

  raise ConfigError('#define "{}" is either\n'.format(name) +
                    "  not present in file {} OR\n".format(path) +
                    "  its value is not an integer literal")


def _find_rocm_config(rocm_install_path, required_version=""):

  def rocm_version_numbers_pre_rocm50(version_file):
    if not os.path.exists(version_file):
      return None
    version_numbers = []
    with open(version_file) as f:
      version_string = f.read().strip()
      version_numbers = version_string.split(".")
    major = int(version_numbers[0])
    minor = int(version_numbers[1])
    patch = int(version_numbers[2].split("-")[0])
    return _get_composite_version_number(major, minor, patch)

  def rocm_version_numbers_post_rocm50(version_file):
    if not os.path.exists(version_file):
      return None
    major = _get_header_version(version_file, "ROCM_VERSION_MAJOR")
    minor = _get_header_version(version_file, "ROCM_VERSION_MINOR")
    patch = _get_header_version(version_file, "ROCM_VERSION_PATCH")
    return _get_composite_version_number(major, minor, patch)
  try:
      rocm_header_path, header_version = _find_header([rocm_install_path], "rocm_version.h",
                                                      required_version,
                                                      rocm_version_numbers_post_rocm50,
                                                      _header_paths)
  except ConfigError as e:\
      rocm_header_path, header_version = _find_header([rocm_install_path], "version-dev",
                                                       required_version,
                                                       rocm_version_numbers_pre_rocm50,
                                                       _header_paths)
  rocm_config = {
      "rocm_version_number": header_version,
      "rocm_header_path": os.path.dirname(rocm_header_path)
  }

  return rocm_config


def _find_hipruntime_config(rocm_install_path, required_version=""):

  def hipruntime_version_number(version_file):
    if not os.path.exists(version_file):
      raise ConfigError(
          'HIP Runtime version file "{}" not found'.format(version_file))
    # This header file has an explicit #define for HIP_VERSION, whose value
    # is (HIP_VERSION_MAJOR * 100 + HIP_VERSION_MINOR)
    # Retreive the major + minor and re-calculate here, since we do not
    # want get into the business of parsing arith exprs
    major = _get_header_version(version_file, "HIP_VERSION_MAJOR")
    minor = _get_header_version(version_file, "HIP_VERSION_MINOR")
    return 100 * major + minor
  header_path, header_version = _find_header([rocm_install_path],
                                             "hip/include/hip/hip_version.h",
                                             required_version,
                                             hipruntime_version_number,
                                             _header_paths)
  library_path = _find_library([rocm_install_path], "amdhip64",
                               required_version, _library_paths)
  library_path = _get_path_if_link(library_path)
  hipruntime_config = {
      "hipruntime_include_dir": os.path.dirname(header_path),
      "hipruntime_version_number": header_version,
      "hipruntime_library_dir": os.path.dirname(library_path)
  }

  return hipruntime_config


def _find_miopen_config(rocm_install_path, required_version=''):

  def miopen_version_numbers(version_file):
    if not os.path.exists(version_file):
      raise ConfigError(
          'MIOpen version file "{}" not found'.format(version_file))
    major = _get_header_version(version_file, "MIOPEN_VERSION_MAJOR")
    minor = _get_header_version(version_file, "MIOPEN_VERSION_MINOR")
    patch = _get_header_version(version_file, "MIOPEN_VERSION_PATCH")
    return _get_composite_version_number(major, minor, patch)

  header_path, header_version = _find_header([rocm_install_path],
                                              "miopen/include/miopen/version.h",
                                              required_version,
                                              miopen_version_numbers,
                                              _header_paths)
  library_path = _find_library([rocm_install_path], "MIOpen",
                               required_version, _library_paths)
  library_path = _get_path_if_link(library_path)
  miopen_config = {
      "miopen_version_number": header_version,
      "miopen_include_dir": os.path.dirname(header_path),
      "miopen_library_dir": os.path.dirname(library_path)
  }

  return miopen_config

def _find_migraphx_config(rocm_install_path, required_version=''):

  def migraphx_version_numbers(version_file):
    if not os.path.exists(version_file):
      raise ConfigError(
          'MIGraphX version file "{}" not found'.format(version_file))
    major = _get_header_version(version_file, "MIGRAPHX_VERSION_MAJOR")
    minor = _get_header_version(version_file, "MIGRAPHX_VERSION_MINOR")
    patch = 0
    return _get_composite_version_number(major, minor, patch)

  header_path, header_version = _find_header([rocm_install_path],
                                              "migraphx/include/migraphx/version.h",
                                              required_version,
                                              migraphx_version_numbers,
                                              _header_paths)
  library_path = _find_library([rocm_install_path], "migraphx",
                               required_version, _library_paths)
  library_path = _get_path_if_link(library_path)
  migraphx_config = {
      "migraphx_version_number": header_version,
      "migraphx_include_dir": os.path.dirname(header_path),
      "migraphx_library_dir": os.path.dirname(library_path)
  }

  return migraphx_config

def _find_hipblas_config(rocm_install_path, required_version=''):

  def hipblas_version_numbers(version_file):
    if not os.path.exists(version_file):
      raise ConfigError(
          'hipblas version file "{}" not found'.format(version_file))
    major = _get_header_version(version_file, "hipblasVersionMajor")
    # ToDo: Fix typo in "hipblaseVersionMinor" with switching to ROCM 5.1.0 or higher, where the typo is fixed
    minor = _get_header_version(version_file, "hipblaseVersionMinor")
    patch = _get_header_version(version_file, "hipblasVersionPatch")
    return _get_composite_version_number(major, minor, patch)
  header_path, header_version = _find_header([rocm_install_path],
                                              "hipblas/include/hipblas-version.h",
                                              required_version,
                                              hipblas_version_numbers,
                                              _header_paths)
  library_path = _find_library([rocm_install_path], "hipblas",
                               required_version, _library_paths)
  library_path = _get_path_if_link(library_path)

  hipblas_config = {
      "hipblas_version_number": header_version,
      "hipblas_include_dir": os.path.dirname(header_path),
      "hipblas_library_dir": os.path.dirname(library_path)
  }

  return hipblas_config

def _find_rocblas_config(rocm_install_path, required_version=''):

  def rocblas_version_numbers(version_file):
    if not os.path.exists(version_file):
      raise ConfigError(
          'rocblas version file "{}" not found'.format(version_file))
    major = _get_header_version(version_file, "ROCBLAS_VERSION_MAJOR")
    minor = _get_header_version(version_file, "ROCBLAS_VERSION_MINOR")
    patch = _get_header_version(version_file, "ROCBLAS_VERSION_PATCH")
    return _get_composite_version_number(major, minor, patch)
  header_path, header_version = _find_header([rocm_install_path],
                                              "rocblas/include/internal/rocblas-version.h",
                                              required_version,
                                              rocblas_version_numbers,
                                              _header_paths)
  library_path = _find_library([rocm_install_path], "rocblas",
                               required_version, _library_paths)
  library_path = _get_path_if_link(library_path)

  rocblas_config = {
      "rocblas_version_number": header_version,
      "rocblas_include_dir": os.path.dirname(header_path),
      "rocblas_library_dir": os.path.dirname(library_path)
  }

  return rocblas_config

def find_rocm_config():
  """Returns a dictionary of ROCm components config info."""
  rocm_install_path = _get_rocm_install_path()
  if not os.path.exists(rocm_install_path):
    raise ConfigError(
        'Specified ROCM_PATH "{}" does not exist'.format(rocm_install_path))

  result = {}

  result["rocm_toolkit_path"] = rocm_install_path
  result.update(_find_rocm_config(rocm_install_path, os.environ.get("TF_ROCM_VERSION", "")))
  result.update(_find_hipruntime_config(rocm_install_path, os.environ.get("TF_HIP_VERSION", "")))
  result.update(_find_miopen_config(rocm_install_path, os.environ.get("TF_MIOPEN_VERSION", "")))
  result.update(_find_migraphx_config(rocm_install_path, os.environ.get("TF_MIGRAPHX_VERSION", "")))
  result.update(_find_hipblas_config(rocm_install_path, os.environ.get("TF_HIPBLAS_VERSION", "")))
  result.update(_find_rocblas_config(rocm_install_path, os.environ.get("TF_ROCBLAS_VERSION", "")))

  return result


def main():
  try:
    for key, value in sorted(find_rocm_config().items()):
      print("%s: %s" % (key, value))
  except ConfigError as e:
    sys.stderr.write("\nERROR: {}\n\n".format(str(e)))
    sys.exit(1)


if __name__ == "__main__":
  main()
