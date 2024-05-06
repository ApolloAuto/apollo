"""Loads the gflags library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "nvjpeg",
        build_file = clean_dep("//third_party/nvjpeg:nvjpeg.BUILD"),
        path = "/usr/src",
    )