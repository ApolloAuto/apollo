load("//tools:common.bzl", "basename", "dirname")
load("//tools/platform:common.bzl", "execute", "make_copy_dir_rule", "make_copy_files_rule")

_APOLLO_SYSROOT_DIR = "APOLLO_SYSROOT_DIR"

_APOLLO_VTK_LIBS = [
    "vtkCommonDataModel",
    "vtkCommonCore",
    "vtkCommonMath",
    "vtkCommonSystem",
    "vtkCommonMisc",
    "vtkCommonTransforms",
    "vtksys",
]

def _vtk_solib_name(basename, version = None):
    """Constructs Linux-specific name of vtk libraries"""
    version = "" if not version else "-" + version
    return "lib{}{}.so".format(basename, version)

def find_sysroot_dir(repository_ctx):
    if _APOLLO_SYSROOT_DIR in repository_ctx.os.environ:
        return repository_ctx.os.environ[_APOLLO_SYSROOT_DIR].strip()
    return None
    # fail("Environment variable APOLLO_SYSROOT_DIR was not specified." +
    #    "Re-run ./apollo6.sh config")

def _vtk_version_from_incl_path(incl_path):
    return basename(incl_path).strip("vtk-")

def _create_local_vtk_repository(repository_ctx):
    sysroot_dir = find_sysroot_dir(repository_ctx)
    result = _vtk_match_version(repository_ctx, sysroot_dir)
    if result == None:
        fail("Oops, Package vtk not found.")
    (version, incl_dir, lib_path) = result

    # Copy the library and header files.
    libraries = [_vtk_solib_name(lib, version) for lib in _APOLLO_VTK_LIBS]

    _lib_path = lib_path + "/"

    # headers   = _get_vtk_headers(incl_dir)
    _incl_dir = incl_dir + "/"
    copy_rules = [
        make_copy_files_rule(
            repository_ctx,
            name = "vtk_lib",
            srcs = [_lib_path + lib for lib in libraries],
            outs = ["vtk/lib/" + lib for lib in libraries],
        ),
        make_copy_dir_rule(
            repository_ctx,
            name = "vtk_include",
            src_dir = incl_dir,
            out_dir = "vtk/include",
        ),
    ]

    # Set up BUILD file.
    build_tpl = repository_ctx.path(Label("//third_party/vtk:BUILD.tpl"))
    repository_ctx.template("BUILD", build_tpl, {
        "%{copy_rules}": "\n".join(copy_rules),
    })

def _vtk_match_version(repository_ctx, sysroot_dir = None):
    cmd = """ldconfig -p | awk -F'=>' '/libvtkCommonCore-.*.so$/ {print $2}'"""
    lib_result = execute(
        repository_ctx,
        ["sh", "-c", cmd],
        empty_stdout_fine = False,
    ).stdout.strip()

    libdict = {}
    for solib in lib_result.split("\n"):
        libpath = dirname(solib)
        version = basename(solib).rstrip(".so").split("-")[-1]
        prefix = solib[:solib.rfind("/lib/")]
        libdict[solib] = (libpath, version, prefix)

    prefix_dirs = ["/usr", "/usr/local", "/opt/apollo/neo/packages/3rd-vtk/latest"]
    if sysroot_dir:
        prefix_dirs.append(sysroot_dir)

    for prefix in reversed(prefix_dirs):
        cmd = """ls -d {}/include/vtk-* 2>/dev/null""".format(prefix)
        incl_dir = execute(
            repository_ctx,
            ["sh", "-c", cmd],
            empty_stdout_fine = True,
        ).stdout.strip()
        if not incl_dir:
            continue

        version = _vtk_version_from_incl_path(incl_dir)
        for k in libdict:
            (lib_path, lib_version, lib_prefix) = libdict[k]
            if lib_version == version:
                return (version, incl_dir, lib_path)

    return None

def _vtk_configure_impl(repository_ctx):
    # Room for _create_remote_vtk_repository
    _create_local_vtk_repository(repository_ctx)

vtk_configure = repository_rule(
    implementation = _vtk_configure_impl,
    environ = [],
)

"""Detects and configures the local vtk library.
Add the following to your WORKSPACE FILE:

```python
vtk_configure(name = "local_config_vtk")
```

Args:
  name: A unique name for this workspace rule.
"""
