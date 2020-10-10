"""Loads the poco library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # poco
    native.new_local_repository(
        name = "poco",
        build_file = clean_dep("//third_party/poco:poco.BUILD"),
        path = "/opt/apollo/sysroot/include",
    )
