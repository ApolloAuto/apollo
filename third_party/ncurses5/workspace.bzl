"""Loads the ncurses5 library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

# Installed via libncurses5-dev
def repo():
    # ncurses5
    native.new_local_repository(
        name = "ncurses5",
        build_file = clean_dep("//third_party/ncurses5:ncurses.BUILD"),
        path = "/usr/include",
    )
