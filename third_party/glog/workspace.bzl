"""Loads the glog library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

# TODO(infra):
# Remove the build_file settings when glog issue #53 was resolved
# Link: https://github.com/google/glog/issues/53#issuecomment-136418497

def repo():
    http_archive(
        name = "com_github_google_glog",
        sha256 = "f28359aeba12f30d73d9e4711ef356dc842886968112162bc73002645139c39c",
        strip_prefix = "glog-0.4.0",
        urls = [
            "https://apollo-system.cdn.bcebos.com/archive/6.0/v0.4.0.tar.gz",
            "https://github.com/google/glog/archive/v0.4.0.tar.gz",
        ],
        build_file = clean_dep("//third_party/glog:glog.BUILD"),
    )
