"""Loads the opengl library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # opengl
    native.new_local_repository(
        name = "opengl",
        build_file = clean_dep("//third_party/opengl:opengl.BUILD"),
        path = "/usr/include",
    )
