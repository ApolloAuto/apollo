"""Loads the qt library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # qt5
    native.new_local_repository(
        name = "qt",
        build_file = clean_dep("//third_party/qt5:qt.BUILD"),
        path = "/usr/local/qt5/include",
    )
