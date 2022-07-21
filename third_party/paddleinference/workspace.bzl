"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "paddleinference",
        sha256 = "5f3ec34d85842d11494b3d2781923dd7103937ae1b1719e2de783b9dbe71a506",
        strip_prefix = "paddleinference",
        urls = [
            "https://apollo-system.bj.bcebos.com/archive/8.0/paddleinference.tar.gz",
        ],
    )
