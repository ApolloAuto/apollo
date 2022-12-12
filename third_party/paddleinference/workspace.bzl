"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "paddleinference",
        sha256 = "a712be880d8b28bbf8aa65d9d846999b13f1b3953880e34fbbd148e9563f3262",
        strip_prefix = "paddleinference",
        urls = [
            "https://apollo-system.bj.bcebos.com/archive/v8.0_bev/paddleinference.tar.gz",
        ],
    )
