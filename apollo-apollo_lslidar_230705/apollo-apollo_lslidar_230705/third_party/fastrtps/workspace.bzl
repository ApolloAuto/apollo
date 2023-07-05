"""Load the fastrtps library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "fastcdr",
        build_file = clean_dep("//third_party/fastrtps:fastcdr.BUILD"),
        path = "/usr/local/fast-rtps/include",
    )

    native.new_local_repository(
        name = "fastrtps",
        build_file = clean_dep("//third_party/fastrtps:fastrtps.BUILD"),
        path = "/usr/local/fast-rtps/include",
    )
