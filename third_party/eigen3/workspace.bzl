"""Loads the eigen library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "eigen",
        build_file = clean_dep("//third_party/eigen3:eigen.BUILD"),
        sha256 = "146a480b8ed1fb6ac7cd33fec9eb5e8f8f62c3683b3f850094d9d5c35a92419a",
        strip_prefix = "eigen-3.3.8",
        urls = [
            "https://apollo-system.cdn.bcebos.com/archive/6.0/eigen-3.3.8.tar.gz",
            "https://gitlab.com/libeigen/eigen/-/archive/3.3.8/eigen-3.3.8.tar.gz",
        ],
    )
    #native.new_local_repository(
    #    name = "eigen",
    #    build_file = clean_dep("//third_party/eigen3:eigen.BUILD"),
    #    path = "/usr/include/eigen3",
    #)
