"""Loads the cpplint library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "cpplint",
        build_file = clean_dep("//third_party/cpplint:cpplint.BUILD"),
        sha256 = "5ace9cb77ddfa61bb0135b21d8a3fe62970266588ba41b8a644cd271d3c0676e",
        strip_prefix = "cpplint-1.5.2",
        urls = [
            "https://apollo-system.cdn.bcebos.com/archive/6.0/1.5.2.tar.gz",
            "https://github.com/cpplint/cpplint/archive/1.5.2.tar.gz",
        ],
    )
