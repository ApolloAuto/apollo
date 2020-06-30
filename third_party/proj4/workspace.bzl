"""Loads the proj4 library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # proj4
    native.new_local_repository(
        name = "proj4",
        build_file = clean_dep("//third_party/proj4:proj4.BUILD"),
        path = "/usr/include",
    )
