"""Repository rule for ROCm autoconfiguration.

`rocm_configure` depends on the following environment variables:

  * `TF_NEED_ROCM`: Whether to enable building with ROCm.
  * `GCC_HOST_COMPILER_PATH`: The GCC host compiler path
  * `ROCM_PATH`: The path to the ROCm toolkit. Default is `/opt/rocm`.
  * `TF_ROCM_AMDGPU_TARGETS`: The AMDGPU targets.
"""

load(
    "//tools/platform:common.bzl",
    "err_out",
    "execute",
    "files_exist",
    "get_bash_bin",
    "get_cpu_value",
    "get_host_environ",
    "get_crosstool_verbose",
    "get_python_bin",
    "raw_exec",
    "realpath",
    "to_list_of_strings",
    "which",
    "make_copy_dir_rule",
    "make_copy_files_rule",
    "tpl_gpus_path",
    "tpl_gpus",
)

_GCC_HOST_COMPILER_PATH = "GCC_HOST_COMPILER_PATH"
_GCC_HOST_COMPILER_PREFIX = "GCC_HOST_COMPILER_PREFIX"
_TF_NEED_ROCM = "TF_NEED_ROCM"
_ROCM_TOOLKIT_PATH = "ROCM_PATH"
_TF_ROCM_AMDGPU_TARGETS = "TF_ROCM_AMDGPU_TARGETS"
_TF_ROCM_CONFIG_REPO = "TF_ROCM_CONFIG_REPO"

def verify_build_defines(params):
    """Verify all variables that crosstool/BUILD.rocm.tpl expects are substituted.

    Args:
      params: dict of variables that will be passed to the BUILD.rocm.tpl template.
    """
    missing = []
    for param in [
        "cxx_builtin_include_directories",
        "extra_no_canonical_prefixes_flags",
        "host_compiler_path",
        "host_compiler_prefix",
        "linker_bin_path",
        "unfiltered_compile_flags",
    ]:
        if ("%{" + param + "}") not in params:
            missing.append(param)

    if missing:
        auto_configure_fail(
            "BUILD.rocm.tpl template is missing these variables: " +
            str(missing) +
            ".\nWe only got: " +
            str(params) +
            ".",
        )

def find_cc(repository_ctx):
    """Find the C++ compiler."""

    target_cc_name = "gcc"
    cc_path_envvar = _GCC_HOST_COMPILER_PATH
    cc_name = target_cc_name

    cc_name_from_env = get_host_environ(repository_ctx, cc_path_envvar)
    if cc_name_from_env:
        cc_name = cc_name_from_env
    if cc_name.startswith("/"):
        # Absolute path, maybe we should make this supported by our which function.
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

def _get_cxx_inc_directories_impl(repository_ctx, cc, lang_is_cpp):
    """Compute the list of default C or C++ include directories."""
    if lang_is_cpp:
        lang = "c++"
    else:
        lang = "c"

    # TODO: We pass -no-canonical-prefixes here to match the compiler flags,
    #       but in rocm_clang CROSSTOOL file that is a `feature` and we should
    #       handle the case when it's disabled and no flag is passed
    result = raw_exec(repository_ctx, [
        cc,
        "-no-canonical-prefixes",
        "-E",
        "-x" + lang,
        "-",
        "-v",
    ])
    stderr = err_out(result)
    index1 = stderr.find(_INC_DIR_MARKER_BEGIN)
    if index1 == -1:
        return []
    index1 = stderr.find("\n", index1)
    if index1 == -1:
        return []
    index2 = stderr.rfind("\n ")
    if index2 == -1 or index2 < index1:
        return []
    index2 = stderr.find("\n", index2 + 1)
    if index2 == -1:
        inc_dirs = stderr[index1 + 1:]
    else:
        inc_dirs = stderr[index1 + 1:index2].strip()

    return [
        str(repository_ctx.path(_cxx_inc_convert(p)))
        for p in inc_dirs.split("\n")
    ]

def get_cxx_inc_directories(repository_ctx, cc):
    """Compute the list of default C and C++ include directories."""

    # For some reason `clang -xc` sometimes returns include paths that are
    # different from the ones from `clang -xc++`. (Symlink and a dir)
    # So we run the compiler with both `-xc` and `-xc++` and merge resulting lists
    includes_cpp = _get_cxx_inc_directories_impl(repository_ctx, cc, True)
    includes_c = _get_cxx_inc_directories_impl(repository_ctx, cc, False)

    includes_cpp_set = depset(includes_cpp)
    return includes_cpp + [
        inc
        for inc in includes_c
        if inc not in includes_cpp_set.to_list()
    ]

def auto_configure_fail(msg):
    """Output failure message when ROCm configuration fails."""
    red = "\033[0;31m"
    no_color = "\033[0m"
    fail("\n%sROCm Configuration Error:%s %s\n" % (red, no_color, msg))

def auto_configure_warning(msg):
    """Output warning message during auto configuration."""
    yellow = "\033[1;33m"
    no_color = "\033[0m"
    print("\n%sAuto-Configuration Warning:%s %s\n" % (yellow, no_color, msg))

# END cc_configure common functions.

def _rocm_include_path(repository_ctx, rocm_config, bash_bin):
    """Generates the cxx_builtin_include_directory entries for rocm inc dirs.

    Args:
      repository_ctx: The repository context.
      rocm_config: The path to the gcc host compiler.

    Returns:
      A string containing the Starlark string for each of the gcc
      host compiler include directories, which can be added to the CROSSTOOL
      file.
    """
    inc_dirs = []
    rocm_toolkit_path = realpath(repository_ctx, rocm_config.rocm_toolkit_path, bash_bin)

    # Add HSA headers (needs to match $HSA_PATH)
    inc_dirs.append(rocm_toolkit_path + "/hsa/include")

    # Add HIP headers (needs to match $HIP_PATH)
    inc_dirs.append(rocm_toolkit_path + "/hip/include")

    # Add HIP-Clang headers (realpath relative to compiler binary)
    inc_dirs.append(rocm_toolkit_path + "/llvm/lib/clang/14.0.0/include")
    inc_dirs.append(rocm_toolkit_path + "/llvm/lib/clang/15.0.0/include")

    return inc_dirs

def _enable_rocm(repository_ctx):
    enable_rocm = get_host_environ(repository_ctx, "TF_NEED_ROCM")
    if enable_rocm == "1":
        if get_cpu_value(repository_ctx) != "Linux":
            auto_configure_warning("ROCm configure is only supported on Linux")
            return False
        return True
    return False

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

def _hipcc_env(repository_ctx):
    """Returns the environment variable string for hipcc.

    Args:
        repository_ctx: The repository context.

    Returns:
        A string containing environment variables for hipcc.
    """
    hipcc_env = ""
    for name in [
        "HIP_CLANG_PATH",
        "DEVICE_LIB_PATH",
        "HIP_VDI_HOME",
        "HIPCC_VERBOSE",
        "HIPCC_COMPILE_FLAGS_APPEND",
        "HIPPCC_LINK_FLAGS_APPEND",
        "HCC_AMDGPU_TARGET",
        "HIP_PLATFORM",
    ]:
        env_value = get_host_environ(repository_ctx, name)
        if env_value:
            hipcc_env = (hipcc_env + " " + name + "=\"" + env_value + "\";")
    return hipcc_env.strip()

def lib_name(lib, version = "", static = False):
    """Constructs the name of a library on Linux.

    Args:
      lib: The name of the library, such as "hip"
      version: The version of the library.
      static: True the library is static or False if it is a shared object.

    Returns:
      The platform-specific name of the library.
    """
    if static:
        return "lib%s.a" % lib
    else:
        if version:
            version = ".%s" % version
        return "lib%s.so%s" % (lib, version)

def _rocm_lib_paths(repository_ctx, lib, basedir):
    file_name = lib_name(lib, version = "", static = False)
    return [
        repository_ctx.path("%s/lib64/%s" % (basedir, file_name)),
        repository_ctx.path("%s/lib64/stubs/%s" % (basedir, file_name)),
        repository_ctx.path("%s/lib/x86_64-linux-gnu/%s" % (basedir, file_name)),
        repository_ctx.path("%s/lib/%s" % (basedir, file_name)),
        repository_ctx.path("%s/%s" % (basedir, file_name)),
    ]

def _batch_files_exist(repository_ctx, libs_paths, bash_bin):
    all_paths = []
    for _, lib_paths in libs_paths:
        for lib_path in lib_paths:
            all_paths.append(lib_path)
    return files_exist(repository_ctx, all_paths, bash_bin)

def _select_rocm_lib_paths(repository_ctx, libs_paths, bash_bin):
    test_results = _batch_files_exist(repository_ctx, libs_paths, bash_bin)

    libs = {}
    i = 0
    for name, lib_paths in libs_paths:
        selected_path = None
        for path in lib_paths:
            if test_results[i] and selected_path == None:
                # For each lib select the first path that exists.
                selected_path = path
            i = i + 1
        if selected_path == None:
            auto_configure_fail("Cannot find rocm library %s" % name)

        libs[name] = struct(file_name = selected_path.basename, path = realpath(repository_ctx, selected_path, bash_bin))

    return libs

def _find_libs(repository_ctx, rocm_config, hipfft_or_rocfft, bash_bin):
    """Returns the ROCm libraries on the system.

    Args:
      repository_ctx: The repository context.
      rocm_config: The ROCm config as returned by _get_rocm_config
      bash_bin: the path to the bash interpreter

    Returns:
      Map of library names to structs of filename and path
    """
    libs_paths = [
        (name, _rocm_lib_paths(repository_ctx, name, path))
        for name, path in [
            ("amdhip64", rocm_config.rocm_toolkit_path + "/hip"),
            ("hipblas", rocm_config.rocm_toolkit_path + "/hipblas"),
            ("rocblas", rocm_config.rocm_toolkit_path + "/rocblas"),
            ("MIOpen", rocm_config.rocm_toolkit_path + "/miopen"),
            ("migraphx", rocm_config.rocm_toolkit_path + "/migraphx"),
        ]
    ]
    return _select_rocm_lib_paths(repository_ctx, libs_paths, bash_bin)

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
        migraphx_version_number: The version of MIGraphX on the system.
        hipruntime_version_number: The version of HIP Runtime on the system.
    """
    config = find_rocm_config(repository_ctx, find_rocm_config_script)
    rocm_toolkit_path = config["rocm_toolkit_path"]
    rocm_version_number = config["rocm_version_number"]
    miopen_version_number = config["miopen_version_number"]
    migraphx_version_number = config["migraphx_version_number"]
    hipruntime_version_number = config["hipruntime_version_number"]
    return struct(
        amdgpu_targets = _amdgpu_targets(repository_ctx, rocm_toolkit_path, bash_bin),
        rocm_toolkit_path = rocm_toolkit_path,
        rocm_version_number = rocm_version_number,
        miopen_version_number = miopen_version_number,
        migraphx_version_number = migraphx_version_number,
        hipruntime_version_number = hipruntime_version_number,
        config = config
    )

_DUMMY_CROSSTOOL_BZL_FILE = """
def error_gpu_disabled():
  fail("ERROR: Building with --config=rocm but Apollo is not configured " +
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
    # Set up build_defs.bzl file for rocm/
    tpl_gpus(
        repository_ctx,
        "rocm:build_defs.bzl",
        {
            "%{rocm_is_configured}": "False",
            "%{rocm_extra_copts}": "[]",
            "%{rocm_gpu_architectures}": "[]",
            "%{rocm_version_number}": "0",
        },
    )

    # Set up BUILD file for rocm/
    tpl_gpus(
        repository_ctx,
        "rocm:BUILD",
        {
            "%{hip_lib}": lib_name("hip"),
            "%{hipblas_lib}": lib_name("hipblas"),
            "%{rocblas_lib}": lib_name("rocblas"),
            "%{miopen_lib}": lib_name("miopen"),
            "%{migraphx_lib}": lib_name("migraphx"),
            "%{migraphx_c_lib}": lib_name("migraphx_c"),
            "%{migraphx_tf_lib}": lib_name("migraphx_tf"),
            "%{migraphx_device_lib}": lib_name("migraphx_device"),
            "%{migraphx_gpu_lib}": lib_name("migraphx_gpu"),
            "%{migraphx_ref_lib}": lib_name("migraphx_ref"),
            "%{migraphx_onnx_lib}": lib_name("migraphx_onnx"),
            "%{copy_rules}": "",
            "%{rocm_headers}": "",
        },
    )

    # Create dummy files for the ROCm toolkit since they are still required by
    # tensorflow/core/platform/default/build_config:rocm.
    repository_ctx.file("rocm/hip/include/hip/hip_runtime.h", "")

    # Set up rocm_config.h, which is used by
    # tensorflow/stream_executor/dso_loader.cc.
    tpl_gpus(
        repository_ctx,
        "rocm:rocm_config.h",
        {
            "%{rocm_amdgpu_targets}": "",
            "%{rocm_toolkit_path}": "",
            "%{rocm_version_number}": "",
            "%{miopen_version_number}": "",
            "%{migraphx_version_number}": "",
            "%{hipruntime_version_number}": "",
            "%{hipblas_version_number}": "",
            "%{rocblas_version_number}": "",
        },
        "rocm/rocm/rocm_config.h",
    )

    # Set up rocm_config.py, which is used by gen_build_info to provide
    # static build environment info to the API
    tpl_gpus(
        repository_ctx,
        "rocm:rocm_config.py",
        {
            "%{rocm_config}": str({}),
        },
        "rocm/rocm/rocm_config.py",
    )

    # If rocm_configure is not configured to build with GPU support, and the user
    # attempts to build with --config=rocm, add a dummy build rule to intercept
    # this and fail with an actionable error message.
    repository_ctx.file(
        "crosstool/error_gpu_disabled.bzl",
        _DUMMY_CROSSTOOL_BZL_FILE,
    )
    repository_ctx.file("crosstool/BUILD", _DUMMY_CROSSTOOL_BUILD_FILE)

def _compute_rocm_extra_copts(repository_ctx, amdgpu_targets):
    copts = []
    for amdgpu_target in amdgpu_targets:
        copts.append("--offload-arch=%s" % amdgpu_target)
    return str(copts)

def _create_local_rocm_repository(repository_ctx):
    """Creates the repository containing files set up to build with ROCm."""

    tpl_paths = {labelname: tpl_gpus_path(repository_ctx, labelname) for labelname in [
        "rocm:build_defs.bzl",
        "crosstool:BUILD.rocm",
        "crosstool:hipcc_cc_toolchain_config.bzl",
        "crosstool:clang/bin/crosstool_wrapper_driver_is_clang_for_hip",
        "rocm:rocm_config.h",
        "rocm:rocm_config.py",
    ]}
    tpl_paths["rocm:BUILD"] = tpl_gpus_path(repository_ctx, "rocm:BUILD")
    find_rocm_config_script = repository_ctx.path(Label("//third_party/gpus:find_rocm_config.py.gz.base64"))

    bash_bin = get_bash_bin(repository_ctx)
    rocm_config = _get_rocm_config(repository_ctx, bash_bin, find_rocm_config_script)

    rocm_lib_srcs = [rocm_config.config["hipruntime_library_dir"] + "/" + lib_name("amdhip64"),
                     rocm_config.config["miopen_library_dir"] + "/" + lib_name("MIOpen"),
                     rocm_config.config["migraphx_library_dir"] + "/" + lib_name("migraphx"),
                     rocm_config.config["migraphx_library_dir"] + "/" + lib_name("migraphx_c"),
                     rocm_config.config["migraphx_library_dir"] + "/" + lib_name("migraphx_tf"),
                     rocm_config.config["migraphx_library_dir"] + "/" + lib_name("migraphx_device"),
                     rocm_config.config["migraphx_library_dir"] + "/" + lib_name("migraphx_gpu"),
                     rocm_config.config["migraphx_library_dir"] + "/" + lib_name("migraphx_ref"),
                     rocm_config.config["migraphx_library_dir"] + "/" + lib_name("migraphx_onnx"),
                     rocm_config.config["hipblas_library_dir"] + "/" + lib_name("hipblas"),
                     rocm_config.config["rocblas_library_dir"] + "/" + lib_name("rocblas")]
    rocm_lib_outs = ["rocm/lib/" + lib_name("amdhip64"),
                     "rocm/lib/" + lib_name("MIOpen"),
                     "rocm/lib/" + lib_name("migraphx"),
                     "rocm/lib/" + lib_name("migraphx_c"),
                     "rocm/lib/" + lib_name("migraphx_tf"),
                     "rocm/lib/" + lib_name("migraphx_device"),
                     "rocm/lib/" + lib_name("migraphx_gpu"),
                     "rocm/lib/" + lib_name("migraphx_ref"),
                     "rocm/lib/" + lib_name("migraphx_onnx"),
                     "rocm/lib/" + lib_name("hipblas"),
                     "rocm/lib/" + lib_name("rocblas")]
    clang_offload_bundler_path = rocm_config.rocm_toolkit_path + "/llvm/bin/clang-offload-bundler"

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
            name = "migraphx-include",
            src_dir =  rocm_config.rocm_toolkit_path + "/migraphx/include",
            out_dir = "rocm/include/migraphx",
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
        "%{rocblas_lib}": lib_name("rocblas"),
        "%{miopen_lib}": lib_name("MIOpen"),
        "%{migraphx_lib}": lib_name("migraphx"),
        "%{migraphx_c_lib}": lib_name("migraphx_c"),
        "%{migraphx_tf_lib}": lib_name("migraphx_tf"),
        "%{migraphx_device_lib}": lib_name("migraphx_device"),
        "%{migraphx_gpu_lib}": lib_name("migraphx_gpu"),
        "%{migraphx_ref_lib}": lib_name("migraphx_ref"),
        "%{migraphx_onnx_lib}": lib_name("migraphx_onnx"),
        "%{copy_rules}": "\n".join(copy_rules),
        "%{rocm_headers}": ('":rocm-include"')
    }

    # Set up BUILD file for rocm/
    tpl_gpus(
        repository_ctx,
        "rocm:BUILD",
        repository_dict,
    )

    repository_ctx.template(
        "rocm/build_defs.bzl",
        tpl_paths["rocm:build_defs.bzl"],
        {
            "%{rocm_extra_copts}": _compute_rocm_extra_copts(
                repository_ctx,
                rocm_config.amdgpu_targets,
            ),
        },
    )

    # Set up crosstool/

    cc = find_cc(repository_ctx)

    host_compiler_includes = get_cxx_inc_directories(repository_ctx, cc)

    host_compiler_prefix = get_host_environ(repository_ctx, _GCC_HOST_COMPILER_PREFIX, "/usr/bin")

    rocm_defines = {}

    rocm_defines["%{host_compiler_prefix}"] = host_compiler_prefix

    rocm_defines["%{linker_bin_path}"] = host_compiler_prefix

    # For gcc, do not canonicalize system header paths; some versions of gcc
    # pick the shortest possible path for system includes when creating the
    # .d file - given that includes that are prefixed with "../" multiple
    # time quickly grow longer than the root of the tree, this can lead to
    # bazel's header check failing.
    rocm_defines["%{extra_no_canonical_prefixes_flags}"] = "\"-fno-canonical-system-headers\""

    rocm_defines["%{unfiltered_compile_flags}"] = to_list_of_strings([
        "-DTENSORFLOW_USE_ROCM=1",
        "-D__HIP_PLATFORM_AMD__",
        "-DEIGEN_USE_HIP",
    ])

    rocm_defines["%{host_compiler_path}"] = "clang/bin/crosstool_wrapper_driver_is_clang_for_hip"

    rocm_defines["%{cxx_builtin_include_directories}"] = to_list_of_strings(
        host_compiler_includes + _rocm_include_path(repository_ctx, rocm_config, bash_bin),
    )

    verify_build_defines(rocm_defines)

    # Only expand template variables in the BUILD file
    repository_ctx.template(
        "crosstool/BUILD",
        tpl_paths["crosstool:BUILD.rocm"],
        rocm_defines,
    )

    # No templating of cc_toolchain_config - use attributes and templatize the
    # BUILD file.
    repository_ctx.template(
        "crosstool/hipcc_cc_toolchain_config.bzl",
        tpl_paths["crosstool:hipcc_cc_toolchain_config.bzl"],
    )

    repository_ctx.template(
        "crosstool/clang/bin/crosstool_wrapper_driver_is_clang_for_hip",
        tpl_paths["crosstool:clang/bin/crosstool_wrapper_driver_is_clang_for_hip"],
        {
            "%{cpu_compiler}": str(cc),
            "%{hipcc_path}": rocm_config.rocm_toolkit_path + "/hip/bin/hipcc",
            "%{hipcc_env}": _hipcc_env(repository_ctx),
            "%{rocr_runtime_path}": rocm_config.rocm_toolkit_path + "/lib",
            "%{rocr_runtime_library}": "hsa-runtime64",
            "%{hip_runtime_path}": rocm_config.rocm_toolkit_path + "/hip/lib",
            "%{hip_runtime_library}": "amdhip64",
            "%{crosstool_verbose}": get_crosstool_verbose(repository_ctx),
            "%{gcc_host_compiler_path}": str(cc),
        },
    )

    # Set up rocm_config.h, which is used by
    # tensorflow/stream_executor/dso_loader.cc.
    repository_ctx.template(
       "rocm/rocm/rocm_config.h",
       tpl_paths["rocm:rocm_config.h"],
       {
           "%{rocm_amdgpu_targets}": ",".join(
               ["\"%s\"" % c for c in rocm_config.amdgpu_targets],
           ),
           "%{rocm_toolkit_path}": rocm_config.rocm_toolkit_path,
           "%{rocm_version_number}": rocm_config.rocm_version_number,
           "%{miopen_version_number}": rocm_config.miopen_version_number,
           "%{migraphx_version_number}": rocm_config.migraphx_version_number,
           "%{hipruntime_version_number}": rocm_config.hipruntime_version_number,
           "%{hipblas_version_number}": rocm_config.config["hipblas_version_number"],
       },
    )

    # Set up rocm_config.py, which is used by gen_build_info to provide
    # static build environment info to the API
    repository_ctx.template(
        "rocm/rocm/rocm_config.py",
        tpl_paths["rocm:rocm_config.py"],
        {
            "%{rocm_config}": str(
            {
                "rocm_version": rocm_config.rocm_version_number,
                "hip_version": rocm_config.hipruntime_version_number,
                "rocm_amdgpu_targets": ",".join(
                    ["\"%s\"" % c for c in rocm_config.amdgpu_targets],
                ),
                "cpu_compiler": str(cc),
            }
            ),
        },
    )

def _create_remote_rocm_repository(repository_ctx, remote_config_repo):
    """Creates pointers to a remotely configured repo set up to build with ROCm."""

def _rocm_autoconf_impl(repository_ctx):
    """Implementation of the rocm_autoconf repository rule."""
    if not _enable_rocm(repository_ctx):
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
    _TF_NEED_ROCM,
    _ROCM_TOOLKIT_PATH,
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
