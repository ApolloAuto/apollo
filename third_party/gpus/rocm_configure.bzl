load(
    "//tools/platform:common.bzl",
    "execute",
    "get_bash_bin",    
    "get_cpu_value",
    "get_host_environ",
    "realpath",
)

_ROCM_CONFIG_REPO = "_ROCM_CONFIG_REPO"

_DEFAULT_ROCM_TOOLKIT_PATH = "/opt/rocm"


def _rocm_include_path(repository_ctx, rocm_config, bash_bin):

    inc_dirs = []
    rocm_toolkit_path = realpath(repository_ctx, rocm_config.rocm_toolkit_path, bash_bin)
    # Add HIP headers (needs to match $HIP_PATH)
    inc_dirs.append(rocm_toolkit_path + "/hip/include")
    # Add HIP-Clang headers (realpath relative to compiler binary) 
    cmd = "ls -d -1 %s/llvm/lib/clang/**/include/" % rocm_toolkit_path
    clang_include_dirs = execute(repository_ctx, [bash_bin, "-c", cmd]).stdout.strip().split("\n")
    for include_dir in clang_include_dirs:
        inc_dirs.append(include_dir)
    return inc_dirs

def _enable_rocm(repository_ctx):
    enable_rocm = get_host_environ(repository_ctx, "NEED_ROCM")
    if enable_rocm == "1":
        if get_cpu_value(repository_ctx) != "Linux":
            return False
        return True
    return False

def _get_rocm_config(repository_ctx, bash_bin):
    rocm_toolkit_path = _DEFAULT_ROCM_TOOLKIT_PATH 
    return struct(
        rocm_toolkit_path = rocm_toolkit_path,
    )

def _tpl_path(repository_ctx, labelname):
    return repository_ctx.path(Label("//third_party/gpus/%s.tpl" % labelname))

def _tpl(repository_ctx, tpl, substitutions = {}, out = None):
    if not out:
        out = tpl.replace(":", "/")
    repository_ctx.template(
        out,
        _tpl_path(repository_ctx, tpl),
        substitutions,
    )

def _create_rocm_repository(repository_ctx):
    tpl_paths = {labelname: _tpl_path(repository_ctx, labelname) for labelname in [
        "rocm:build_defs.bzl",
        "rocm:BUILD",
    ]}

    bash_bin = get_bash_bin(repository_ctx)
    rocm_config = _get_rocm_config(repository_ctx, bash_bin)

    repository_ctx.template(
        "rocm/build_defs.bzl",
        tpl_paths["rocm:build_defs.bzl"],
        {
            "%{rocm_is_configured}": "True",
        },
    )
    repository_ctx.template(
        "rocm/BUILD",
        tpl_paths["rocm:BUILD"],
    )

_ENVIRONS = [
    "NEED_ROCM",
]

rocm_configure = repository_rule(
    implementation =  _create_rocm_repository,
    environ = _ENVIRONS + [_ROCM_CONFIG_REPO],
)
