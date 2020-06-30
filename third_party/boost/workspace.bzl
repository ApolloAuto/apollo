"""Loads the boost library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "boost",
        build_file = clean_dep("//third_party/boost:boost.BUILD"),
        path = "/opt/apollo/sysroot/include",
    )
