"""Loads the ffmpeg library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "ffmpeg",
        build_file = clean_dep("//third_party/ffmpeg:ffmpeg.BUILD"),
        path = "/opt/apollo/sysroot/include",
    )
