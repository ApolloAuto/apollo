"""Loads the glew library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # gflags
    native.new_local_repository(
        name = "glew",
        build_file = clean_dep("//third_party/glew:glew.BUILD"),
        path = "/usr/include",
    )
