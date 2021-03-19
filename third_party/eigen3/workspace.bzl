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
        sha256 = "a8d87c8df67b0404e97bcef37faf3b140ba467bc060e2b883192165b319cea8d",
        strip_prefix = "eigen-git-mirror-3.3.7",
        urls = [
            "https://apollo-system.cdn.bcebos.com/archive/6.0/3.3.7.tar.gz",
            "https://github.com/eigenteam/eigen-git-mirror/archive/3.3.7.tar.gz",
        ],
    )
    #native.new_local_repository(
    #    name = "eigen",
    #    build_file = clean_dep("//third_party/eigen3:eigen.BUILD"),
    #    path = "/usr/include/eigen3",
    #)
