"""Loads the sqlite3 library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

# apt-get -y install libsqlite3-dev

def repo():
    # sqlite3
    native.new_local_repository(
        name = "sqlite3",
        build_file = clean_dep("//third_party/sqlite3:sqlite3.BUILD"),
        path = "/usr/include",
    )
