"""Loads the local_integ library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # local_integ
    native.new_local_repository(
        name = "local_integ",
        build_file = clean_dep("//third_party/local_integ:local_integ.BUILD"),
        path = "/usr/local/apollo/local_integ",
    )
