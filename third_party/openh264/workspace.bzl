"""Loads the openh264 library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "openh264",
        build_file = clean_dep("//third_party/openh264:openh264.BUILD"),
        # path = "/usr/local/include",
        path = "/opt/apollo/sysroot/include",
    )
