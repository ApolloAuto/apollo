"""Loads the lz4 library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # lz4
    native.new_local_repository(
        name = "lz4",
        build_file = clean_dep("//third_party/lz4:lz4.BUILD"),
        path = "/usr/include",
    )
