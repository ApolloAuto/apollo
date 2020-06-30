"""Loads the libtorch_gpu library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # local_integ
    native.new_local_repository(
        name = "libtorch_gpu",
        build_file = clean_dep("//third_party/libtorch_gpu:libtorch_gpu.BUILD"),
        path = "/usr/local/libtorch_gpu/include",
    )
