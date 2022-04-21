"""Repository rule for ROCm autoconfiguration.

`rocm_configure` depends on the following environment variables:

  * `TF_NEED_HIP`: Whether to enable building with HIP.
  * `GCC_HOST_COMPILER_PATH`: The GCC host compiler path
  * `TF_HIP_CLANG`: Whether to use clang as a HIP compiler (will be a default path).
  * `CLANG_HIP_COMPILER_PATH`: The clang compiler path that will be used for
    both host and device code compilation if TF_HIP_CLANG is 1.
  * `TF_SYSROOT`: The sysroot to use when compiling.
  * `HIP_PATH` (deprecated): The path to the HIP SDK. Default is
    `/opt/rocm/hip`.
  * `TF_HIP_VERSION`: The version of the HIP SDK. If this is blank, then
    use the system default.
  * `TF_MIOPEN_VERSION`: The version of the MIOpen library.
  * `MIOPEN_INSTALL_PATH` (deprecated): The path to the MIOpen library. Default is
    `/opt/rocm/miopen`.
  * `PYTHON_BIN_PATH`: The python binary path
"""

load(
    "//tools/platform:common.bzl",
    "config_repo_label",
    "err_out",
    "execute",
    "get_bash_bin",
    "get_cpu_value",
    "get_host_environ",
    "get_python_bin",
    "raw_exec",
    "read_dir",
    "realpath",
    "which",
)

_GCC_HOST_COMPILER_PATH = "GCC_HOST_COMPILER_PATH"
_GCC_HOST_COMPILER_PREFIX = "GCC_HOST_COMPILER_PREFIX"
_CLANG_HIP_COMPILER_PATH = "CLANG_HIP_COMPILER_PATH"
_TF_SYSROOT = "TF_SYSROOT"
_HIP_PATH = "HIP_PATH"
_TF_HIP_VERSION = "TF_HIP_VERSION"
_TF_MIOPEN_VERSION = "TF_MIOPEN_VERSION"
_MIOPEN_INSTALL_PATH = "MIOPEN_INSTALL_PATH"
_TF_ROCM_CONFIG_REPO = "TF_ROCM_CONFIG_REPO"
_PYTHON_BIN_PATH = "PYTHON_BIN_PATH"
_GPU_PLATFORM = "GPU_PLATFORM"

def to_list_of_strings(elements):
    """Convert the list of ["a", "b", "c"] into '"a", "b", "c"'.

    This is to be used to put a list of strings into the bzl file templates
    so it gets interpreted as list of strings in Starlark.

    Args:
      elements: list of string elements

    Returns:
      single string of elements wrapped in quotes separated by a comma."""
    quoted_strings = ["\"" + element + "\"" for element in elements]
    return ", ".join(quoted_strings)

def verify_build_defines(params):
    """Verify all variables that crosstool/BUILD.tpl expects are substituted.

    Args:
      params: dict of variables that will be passed to the BUILD.tpl template.
    """
    missing = []
    for param in [
        "cxx_builtin_include_directories",
        "extra_no_canonical_prefixes_flags",
        "host_compiler_path",
        "host_compiler_prefix",
        "host_compiler_warnings",
        "linker_bin_path",
        "compiler_deps",
        "unfiltered_compile_flags",
    ]:
        if ("%{" + param + "}") not in params:
            missing.append(param)

    if missing:
        auto_configure_fail(
            "BUILD.tpl template is missing these variables: " +
            str(missing) +
            ".\nWe only got: " +
            str(params) +
            ".",
        )

def _flag_enabled(repository_ctx, flag_name):
    return get_host_environ(repository_ctx, flag_name) == "1"

def _use_hip_clang(repository_ctx):
    return _flag_enabled(repository_ctx, "TF_HIP_CLANG")

def find_cc(repository_ctx):
    """Find the C++ compiler."""
    if _use_hip_clang(repository_ctx):
        target_cc_name = "clang"
        cc_path_envvar = _CLANG_HIP_COMPILER_PATH
    else:
        target_cc_name = "gcc"
        cc_path_envvar = _GCC_HOST_COMPILER_PATH
    cc_name = target_cc_name

    cc_name_from_env = get_host_environ(repository_ctx, cc_path_envvar)
    if cc_name_from_env:
        cc_name = cc_name_from_env
    if cc_name.startswith("/"):
        # Absolute path, maybe we should make this supported by our 'which' function.
        return cc_name
    cc = which(repository_ctx, cc_name)
    if cc == None:
        fail(("Cannot find {}, either correct your path or set the {}" +
              " environment variable").format(target_cc_name, cc_path_envvar))
    return cc

_INC_DIR_MARKER_BEGIN = "#include <...>"

def _cxx_inc_convert(path):
    """Convert path returned by cc -E xc++ in a complete path."""
    return path.strip()

def _normalize_include_path(repository_ctx, path):
    """Normalizes include paths before writing them to the crosstool.

      If path points inside the 'crosstool' folder of the repository, a relative
      path is returned.
      If path points outside the 'crosstool' folder, an absolute path is returned.
      """
    path = str(repository_ctx.path(path))
    crosstool_folder = str(repository_ctx.path(".").get_child("crosstool"))

    if path.startswith(crosstool_folder):
        # We drop the path to "$REPO/crosstool" and a trailing path separator.
        return path[len(crosstool_folder) + 1:]
    return path

def auto_configure_fail(msg):
    """Output failure message when ROCm configuration fails."""
    red = "\033[0;31m"
    no_color = "\033[0m"
    fail("\n%sROCm Configuration Error:%s %s\n" % (red, no_color, msg))

# END cc_configure common functions.

def enable_hip(repository_ctx):
    """Returns whether to build with HIP support."""
    return int(get_host_environ(repository_ctx, "TF_NEED_HIP", False))

def _create_local_rocm_repository(repository_ctx):
    """Creates the repository containing files set up to build with ROCm."""

def _create_remote_rocm_repository(repository_ctx, remote_config_repo):
    """Creates pointers to a remotely configured repo set up to build with ROCm."""
    _tpl(
        repository_ctx,
        "rocm:build_defs.bzl",
        {},
    )
    repository_ctx.template(
        "rocm/BUILD",
        config_repo_label(remote_config_repo, "rocm:BUILD"),
        {},
    )
    repository_ctx.template(
        "rocm/build_defs.bzl",
        config_repo_label(remote_config_repo, "rocm:build_defs.bzl"),
        {},
    )

    repository_ctx.template(
        "crosstool/BUILD",
        config_repo_label(remote_config_repo, "crosstool:BUILD"),
        {},
    )

    repository_ctx.template(
        "crosstool/cc_toolchain_config.bzl",
        config_repo_label(remote_config_repo, "crosstool:cc_toolchain_config.bzl"),
        {},
    )

    repository_ctx.template(
        "crosstool/clang/bin/crosstool_wrapper_driver_is_clang_for_hip",
        config_repo_label(remote_config_repo, "crosstool:clang/bin/crosstool_wrapper_driver_is_clang_for_hip"),
        {},
    )

def _tpl(repository_ctx, tpl, substitutions = {}, out = None):
    if not out:
        out = tpl.replace(":", "/")
    repository_ctx.template(
        out,
        Label("//third_party/gpus/%s.tpl" % tpl),
        substitutions,
    )

_DUMMY_CROSSTOOL_BZL_FILE = """
def error_gpu_disabled():
  fail("ERROR: Building with --config=hip but Apollo is not configured " +
       "to build with GPU support. Please re-run ./scripts/apollo_config.sh" +
       "to build with GPU support.")

  native.genrule(
      name = "error_gen_crosstool",
      outs = ["CROSSTOOL"],
      cmd = "echo 'Should not be run.' && exit 1",
  )

  native.filegroup(
      name = "crosstool",
      srcs = [":CROSSTOOL"],
      output_licenses = ["unencumbered"],
  )
"""

_DUMMY_CROSSTOOL_BUILD_FILE = """
load("//crosstool:error_gpu_disabled.bzl", "error_gpu_disabled")
error_gpu_disabled()
"""

def _create_dummy_repository(repository_ctx):
    # If rocm_configure is not configured to build with GPU support, and the user
    # attempts to build with --config=hip, add a dummy build rule to intercept
    # this and fail with an actionable error message.
    repository_ctx.file(
        "crosstool/error_gpu_disabled.bzl",
        _DUMMY_CROSSTOOL_BZL_FILE,
    )
    repository_ctx.file("crosstool/BUILD", _DUMMY_CROSSTOOL_BUILD_FILE)

def _rocm_autoconf_impl(repository_ctx):
    """Implementation of the rocm_autoconf repository rule."""
    if get_host_environ(repository_ctx, _GPU_PLATFORM) != "AMD":
        repository_ctx.file("crosstool/error_gpu_disabled.bzl", _DUMMY_CROSSTOOL_BZL_FILE)
        repository_ctx.file("crosstool/BUILD", _DUMMY_CROSSTOOL_BUILD_FILE)
        return
    if not enable_hip(repository_ctx):
        _create_dummy_repository(repository_ctx)
    elif get_host_environ(repository_ctx, _TF_ROCM_CONFIG_REPO) != None:
        _create_remote_rocm_repository(
            repository_ctx,
            get_host_environ(repository_ctx, _TF_ROCM_CONFIG_REPO),
        )
    else:
        _create_local_rocm_repository(repository_ctx)

_ENVIRONS = [
    _GCC_HOST_COMPILER_PATH,
    _GCC_HOST_COMPILER_PREFIX,
    _CLANG_HIP_COMPILER_PATH,
    "TF_NEED_HIP",
    "TF_HIP_CLANG",
    _HIP_PATH,
    _MIOPEN_INSTALL_PATH,
    _TF_HIP_VERSION,
    _TF_MIOPEN_VERSION,
    _PYTHON_BIN_PATH,
    _GPU_PLATFORM,
]

rocm_configure = repository_rule(
    implementation = _rocm_autoconf_impl,
    environ = _ENVIRONS + [_TF_ROCM_CONFIG_REPO],
)
"""Detects and configures the local ROCm toolchain.

Add the following to your WORKSPACE FILE:

```python
rocm_configure(name = "local_config_rocm")
```

Args:
  name: A unique name for this workspace rule.
"""
