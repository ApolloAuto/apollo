"""Loads the atlas library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

# Installed via atlas-dev
def repo():
    # atlas
    native.new_local_repository(
        name = "atlas",
        build_file = clean_dep("//third_party/atlas:atlas.BUILD"),
        path = "/usr/include",  # /usr/include/$(uname -m)-linux-gnu
    )
