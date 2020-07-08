"""Loads the pcl library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

# TODO(all): pcl_configure.bzl
def repo():
    # pcl
    native.new_local_repository(
        name = "pcl",
        build_file = clean_dep("//third_party/pcl:pcl.BUILD"),
        path = "/opt/apollo/sysroot/include/pcl-1.10",
    )
