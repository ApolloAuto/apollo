"""Loads the ipopt library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # ipopt
    native.new_local_repository(
        name = "ipopt",
        build_file = clean_dep("//third_party/ipopt:ipopt.BUILD"),
        path = "/usr/include",
    )
