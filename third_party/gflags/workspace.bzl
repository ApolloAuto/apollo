"""Loads the gflags library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # gflags
    native.new_local_repository(
        name = "com_github_gflags_gflags",
        build_file = clean_dep("//third_party/gflags:gflags.BUILD"),
        path = "/usr/local/include/gflags",
    )

#http_archive(
#    name = "com_github_gflags_gflags",
#    build_file = "gflags.BUILD",
#    sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
#    strip_prefix = "gflags-2.2.2",
#    urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
#)
