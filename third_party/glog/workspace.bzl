"""Loads the glog library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # glog
    native.new_local_repository(
        name = "com_google_glog",
        build_file = clean_dep("//third_party/glog:glog.BUILD"),
        path = "/usr/local/include/glog",
    )
