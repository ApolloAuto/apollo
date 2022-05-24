"""Repository rule for ROCm autoconfiguration.

`rocm_configure` depends on the following environment variables:

  * `TF_NEED_ROCM`: Whether to enable building with HIP.
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
  * `TF_ROCM_AMDGPU_TARGETS`: The AMDGPU targets.
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
    "realpath",
    "to_list_of_strings",
    "flag_enabled",
    "which",
    "make_copy_dir_rule",
    "make_copy_files_rule",
)

_GCC_HOST_COMPILER_PATH = "GCC_HOST_COMPILER_PATH"
_GCC_HOST_COMPILER_PREFIX = "GCC_HOST_COMPILER_PREFIX"
_CLANG_HIP_COMPILER_PATH = "CLANG_HIP_COMPILER_PATH"
_TF_SYSROOT = "TF_SYSROOT"
_TF_NEED_ROCM = "TF_NEED_ROCM"
_HIP_PATH = "HIP_PATH"
_TF_HIP_VERSION = "TF_HIP_VERSION"
_TF_MIOPEN_VERSION = "TF_MIOPEN_VERSION"
_MIOPEN_INSTALL_PATH = "MIOPEN_INSTALL_PATH"
_TF_ROCM_AMDGPU_TARGETS = "TF_ROCM_AMDGPU_TARGETS"
_TF_ROCM_CONFIG_REPO = "TF_ROCM_CONFIG_REPO"
_PYTHON_BIN_PATH = "PYTHON_BIN_PATH"
_GPU_PLATFORM = "GPU_PLATFORM"

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

def _use_hip_clang(repository_ctx):
    return flag_enabled(repository_ctx, "TF_HIP_CLANG")

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

def enable_rocm(repository_ctx):
    """Returns whether to build with HIP support."""
    return int(get_host_environ(repository_ctx, _TF_NEED_ROCM, False))

def _amdgpu_targets(repository_ctx, rocm_toolkit_path, bash_bin):
    """Returns a list of strings representing AMDGPU targets."""
    amdgpu_targets_str = get_host_environ(repository_ctx, _TF_ROCM_AMDGPU_TARGETS)
    if not amdgpu_targets_str:
        cmd = "%s/bin/rocm_agent_enumerator" % rocm_toolkit_path
        result = execute(repository_ctx, [bash_bin, "-c", cmd])
        targets = [target for target in result.stdout.strip().split("\n") if target != "gfx000"]
        targets = {x: None for x in targets}
        targets = list(targets.keys())
        amdgpu_targets_str = ",".join(targets)
    amdgpu_targets = amdgpu_targets_str.split(",")
    for amdgpu_target in amdgpu_targets:
        if amdgpu_target[:3] != "gfx":
            auto_configure_fail("Invalid AMDGPU target: %s" % amdgpu_target)
    return amdgpu_targets

def _create_local_rocm_repository(repository_ctx):
    """Creates the repository containing files set up to build with ROCm."""
    find_rocm_config_script = repository_ctx.path(Label("//third_party/gpus:find_rocm_config.py.gz.base64"))

    bash_bin = get_bash_bin(repository_ctx)
    rocm_config = _get_rocm_config(repository_ctx, bash_bin, find_rocm_config_script)

    rocm_lib_srcs = [rocm_config.config["hipruntime_library_dir"] + "/" + lib_name("amdhip64"),
                     rocm_config.config["miopen_library_dir"] + "/" + lib_name("MIOpen"),
                     rocm_config.config["hipblas_library_dir"] + "/" + lib_name("hipblas")]
    rocm_lib_outs = ["rocm/lib/" + lib_name("amdhip64"),
                     "rocm/lib/" + lib_name("MIOpen"),
                     "rocm/lib/" + lib_name("hipblas")]
    clang_offload_bundler_path = rocm_config.rocm_toolkit_path + "/llvm/bin/clang-offload-bundler"

    # Set up build_defs.bzl file for rocm/
    tpl_labelname = "rocm:build_defs.bzl"
    _tpl(
        repository_ctx,
        tpl_labelname,
        {},
        tpl_labelname
    )

    # Set up BUILD file for rocm/
    tpl_labelname = "rocm:BUILD"

    # Copy header and library files to execroot.
    copy_rules = [
        make_copy_dir_rule(
            repository_ctx,
            name = "rocm-include",
            src_dir = rocm_config.config["rocm_header_path"],
            out_dir = "rocm/include",
            exceptions = ["gtest", "gmock"],
        ),
        make_copy_dir_rule(
            repository_ctx,
            name = "miopen-include",
            src_dir =  rocm_config.rocm_toolkit_path + "/miopen/include",
            out_dir = "rocm/include/miopen",
        ),
        make_copy_dir_rule(
            repository_ctx,
            name = "hipblas-include",
            src_dir = rocm_config.rocm_toolkit_path + "/hipblas/include",
            out_dir = "rocm/include/hipblas",
        ),
        make_copy_files_rule(
            repository_ctx,
            name = "rocm-lib",
            srcs = rocm_lib_srcs,
            outs = rocm_lib_outs,
        ),
        make_copy_files_rule(
            repository_ctx,
            name = "rocm-bin",
            srcs = [
                clang_offload_bundler_path,
            ],
            outs = [
                "rocm/bin/" + "clang-offload-bundler",
            ],
        )
    ]

    repository_dict = {
        "%{hip_lib}": lib_name("amdhip64"),
        "%{hipblas_lib}": lib_name("hipblas"),
        "%{miopen_lib}": lib_name("MIOpen"),
        "%{copy_rules}": "\n".join(copy_rules),
        "%{rocm_headers}": ('":rocm-include"')
    }
    _tpl(
        repository_ctx,
        tpl_labelname,
        repository_dict,
        tpl_labelname
    )

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

def _tpl_path(repository_ctx, filename):
    return repository_ctx.path(Label("//third_party/gpus/%s.tpl" % filename))

def _tpl(repository_ctx, tpl, substitutions = {}, out = None):
    if not out == None:
        out = tpl.replace(":", "/")
    repository_ctx.template(
        out,
        Label("//third_party/gpus/%s.tpl" % tpl),
        substitutions,
    )

def _get_rocm_config(repository_ctx, bash_bin, find_rocm_config_script):
    """Detects and returns information about the ROCm installation on the system.

    Args:
      repository_ctx: The repository context.
      bash_bin: the path to the path interpreter

    Returns:
      A struct containing the following fields:
        rocm_toolkit_path: The ROCm toolkit installation directory.
        amdgpu_targets: A list of the system's AMDGPU targets.
        rocm_version_number: The version of ROCm on the system.
        miopen_version_number: The version of MIOpen on the system.
        hipruntime_version_number: The version of HIP Runtime on the system.
    """
    config = find_rocm_config(repository_ctx, find_rocm_config_script)
    rocm_toolkit_path = config["rocm_toolkit_path"]
    rocm_version_number = config["rocm_version_number"]
    miopen_version_number = config["miopen_version_number"]
    hipruntime_version_number = config["hipruntime_version_number"]
    return struct(
        amdgpu_targets = _amdgpu_targets(repository_ctx, rocm_toolkit_path, bash_bin),
        rocm_toolkit_path = rocm_toolkit_path,
        rocm_version_number = rocm_version_number,
        miopen_version_number = miopen_version_number,
        hipruntime_version_number = hipruntime_version_number,
        config = config
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

# TODO(emankov): Bring into compliance with lib_name from cuda_configure.bzl and then move it to /gpus/common.bzl
def lib_name(base_name, version = None, static = False):
    """Constructs the platform-specific name of a library.

      Args:
        base_name: The name of the library, such as "cudart"
        version: The version of the library.
        static: True the library is static or False if it is a shared object.

      Returns:
        The platform-specific name of the library.
      """
    version = "" if not version else "." + version
    if static:
        return "lib%s.a" % base_name
    return "lib%s.so%s" % (base_name, version)

def _create_dummy_repository(repository_ctx):
    # Set up build_defs.bzl file for rocm/
    tpl_labelname = "rocm:build_defs.bzl"
    _tpl(
        repository_ctx,
        tpl_labelname,
        {
            "%{rocm_is_configured}": "False",
            "%{rocm_extra_copts}": "[]",
            "%{rocm_gpu_architectures}": "[]",
            "%{rocm_version_number}": "0",
        },
        tpl_labelname
    )

    # Set up BUILD file for rocm/
    tpl_labelname = "rocm:BUILD"
    _tpl(
        repository_ctx,
        tpl_labelname,
        {
            "%{hip_lib}": lib_name("hip"),
            "%{hipblas_lib}": lib_name("hipblas"),
            "%{miopen_lib}": lib_name("miopen"),
            "%{copy_rules}": "",
            "%{rocm_headers}": "",
        },
        tpl_labelname
    )

    # Create dummy files for the ROCm toolkit since they are still required by
    # tensorflow/core/platform/default/build_config:rocm.
    repository_ctx.file("rocm/hip/include/hip/hip_runtime.h", "")

    # Set up rocm_config.h, which is used by
    # tensorflow/stream_executor/dso_loader.cc.
    # _tpl(
    #     repository_ctx,
    #     "rocm:rocm_config.h",
    #     {
    #         "%{rocm_toolkit_path}": _DEFAULT_ROCM_TOOLKIT_PATH,
    #     },
    #     "rocm/rocm/rocm_config.h",
    # )

    # If rocm_configure is not configured to build with GPU support, and the user
    # attempts to build with --config=rocm, add a dummy build rule to intercept
    # this and fail with an actionable error message.
    repository_ctx.file(
        "crosstool/error_gpu_disabled.bzl",
        _DUMMY_CROSSTOOL_BZL_FILE,
    )
    repository_ctx.file("crosstool/BUILD", _DUMMY_CROSSTOOL_BUILD_FILE)

def _exec_find_rocm_config(repository_ctx, script_path):
    python_bin = get_python_bin(repository_ctx)
    compressed_contents = repository_ctx.read(script_path)
    decompress_and_execute_cmd = (
        "from zlib import decompress;" +
        "from base64 import b64decode;" +
        "from os import system;" +
        "script = decompress(b64decode('%s'));" % compressed_contents +
        "f = open('script.py', 'wb');" +
        "f.write(script);" +
        "f.close();" +
        "system('\"%s\" script.py');" % (python_bin)
    )
    return execute(repository_ctx, [python_bin, "-c", decompress_and_execute_cmd])

def find_rocm_config(repository_ctx, script_path):
    """Returns ROCm config dictionary from running find_rocm_config.py"""
    exec_result = _exec_find_rocm_config(repository_ctx, script_path)
    if exec_result.return_code:
        auto_configure_fail("Failed to run find_rocm_config.py: %s" % err_out(exec_result))
    # Parse the dict from stdout.
    return dict([tuple(x.split(": ")) for x in exec_result.stdout.splitlines()])

def _rocm_autoconf_impl(repository_ctx):
    """Implementation of the rocm_autoconf repository rule."""
    if not enable_rocm(repository_ctx):
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
    _TF_NEED_ROCM,
    "TF_HIP_CLANG",
    _HIP_PATH,
    _MIOPEN_INSTALL_PATH,
    _TF_HIP_VERSION,
    _TF_MIOPEN_VERSION,
    _PYTHON_BIN_PATH,
    _GPU_PLATFORM,
    _TF_ROCM_AMDGPU_TARGETS,
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
