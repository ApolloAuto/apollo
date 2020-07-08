"""Loads the tf2 library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # tf2
    native.new_local_repository(
        name = "tf2",
        build_file = clean_dep("//third_party/tf2:tf2.BUILD"),
        path = "/opt/apollo/pkgs/tf2/include",
    )
