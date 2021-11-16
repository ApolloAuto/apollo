"""Loads the npp library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # npp
    native.new_local_repository(
        name = "npp",
        build_file = clean_dep("//third_party/npp:npp.BUILD"),
        path = "/usr/local/cuda",
    )
