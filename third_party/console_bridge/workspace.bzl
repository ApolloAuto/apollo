"""Loads the console_bridge library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

# Installed via apt-get -y libconsole-bridge-dev
def repo():
    # console_bridge
    native.new_local_repository(
        name = "console_bridge",
        build_file = clean_dep("//third_party/console_bridge:console_bridge.BUILD"),
        path = "/usr/include",
    )
