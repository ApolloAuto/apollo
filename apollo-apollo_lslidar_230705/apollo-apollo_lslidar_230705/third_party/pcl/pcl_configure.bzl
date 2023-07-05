load("//tools:common.bzl", "basename", "dirname")
load("//tools/platform:common.bzl", "execute", "make_copy_dir_rule", "make_copy_files_rule")

_APOLLO_SYSROOT_DIR = "APOLLO_SYSROOT_DIR"

_APOLLO_PCL_LIBS = [
    "pcl_common",
    "pcl_features",
    "pcl_filters",
    "pcl_io_ply",
    "pcl_io",
    "pcl_kdtree",
    "pcl_keypoints",
    "pcl_octree",
    "pcl_outofcore",
    "pcl_people",
    "pcl_recognition",
    "pcl_registration",
    "pcl_sample_consensus",
    "pcl_search",
    "pcl_segmentation",
    "pcl_surface",
    "pcl_tracking",
    "pcl_visualization",
]

def find_sysroot_dir(repository_ctx):
    if _APOLLO_SYSROOT_DIR in repository_ctx.os.environ:
        return repository_ctx.os.environ[_APOLLO_SYSROOT_DIR].strip()
    return None
    # fail("Environment variable APOLLO_SYSROOT_DIR was not specified." +
    #    "Re-run ./apollo6.sh config")

def _pcl_version_from_incl_path(incl_path):
    return basename(incl_path).strip("pcl-")

def _create_local_pcl_repository(repository_ctx):
    sysroot_dir = find_sysroot_dir(repository_ctx)
    result = _pcl_match_version(repository_ctx, sysroot_dir)
    if result == None:
        fail("Oops, Package pcl not found.")
    (version, incl_dir, lib_path) = result

    # Copy the library and header files.
    libraries = ["lib{}.so".format(lib) for lib in _APOLLO_PCL_LIBS]

    _lib_path = lib_path + "/"

    # headers   = _get_pcl_headers(incl_dir)
    _incl_dir = incl_dir + "/"
    copy_rules = [
        make_copy_files_rule(
            repository_ctx,
            name = "pcl_lib",
            srcs = [_lib_path + lib for lib in libraries],
            outs = ["pcl/lib/" + lib for lib in libraries],
        ),
        make_copy_dir_rule(
            repository_ctx,
            name = "pcl_include",
            src_dir = incl_dir,
            out_dir = "pcl/include",
        ),
    ]

    _pcl_linkopts = ["-L{}".format(lib_path)] + ["-l{}".format(lib) for lib in _APOLLO_PCL_LIBS]

    # Set up BUILD file.
    build_tpl = repository_ctx.path(Label("//third_party/pcl:BUILD.tpl"))
    repository_ctx.template("BUILD", build_tpl, {
        "%{pcl_linkopts}": "%s" % (_pcl_linkopts),
        "%{copy_rules}": "\n".join(copy_rules),
    })

def _pcl_match_version(repository_ctx, sysroot_dir = None):
    cmd = """ldconfig -p | awk -F'=>' '/libpcl_common.so$/ {print $2}'"""
    result = execute(
        repository_ctx,
        ["sh", "-c", cmd],
        empty_stdout_fine = False,
    ).stdout.strip()

    solib = result.rstrip("\n")
    lib_path = dirname(solib)
    prefix = solib[:solib.rfind("/lib/")]
    prefix_dirs = ["/usr", "/usr/local", "/opt/apollo/neo"]
    if sysroot_dir:
        prefix_dirs.append(sysroot_dir)
    if prefix not in prefix_dirs:
        return None

    cmd = """ls -d {}/include/pcl-* 2>/dev/null""".format(prefix)
    incl_dir = execute(
        repository_ctx,
        ["sh", "-c", cmd],
        empty_stdout_fine = True,
    ).stdout.strip()
    if not incl_dir:
        return None

    version = _pcl_version_from_incl_path(incl_dir)
    return (version, incl_dir, lib_path)

def _pcl_configure_impl(repository_ctx):
    # Room for _create_remote_pcl_repository
    _create_local_pcl_repository(repository_ctx)

pcl_configure = repository_rule(
    implementation = _pcl_configure_impl,
    environ = [],
)

"""Detects and configures the local pcl library.
Add the following to your WORKSPACE FILE:

```python
pcl_configure(name = "local_config_pcl")
```

Args:
  name: A unique name for this workspace rule.
"""
