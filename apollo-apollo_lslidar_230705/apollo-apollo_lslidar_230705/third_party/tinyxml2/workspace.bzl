"""Loads the tinyxml2 library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # tinyxml2
    native.new_local_repository(
        name = "tinyxml2",
        build_file = clean_dep("//third_party/tinyxml2:tinyxml2.BUILD"),
        path = "/usr/include",
    )
