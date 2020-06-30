"""Loads the libtorch_cpu library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # local_integ
    native.new_local_repository(
        name = "libtorch_cpu",
        build_file = clean_dep("//third_party/libtorch_cpu:libtorch_cpu.BUILD"),
        path = "/usr/local/libtorch_cpu/include",
    )
