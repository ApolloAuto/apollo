"""Loads the osqp library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "osqp",
        build_file = clean_dep("//third_party/osqp:osqp.BUILD"),
        path = "/opt/apollo/sysroot/include",
    )
