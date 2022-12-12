"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "paddleinference",
        sha256 = "4b98cd056de43ca5e2e8a63fdc06f5895437904cedccfc12e43e2415733667d7",
        strip_prefix = "paddleinference",
        urls = [
            "https://apollo-system.bj.bcebos.com/archive/v8.0_bev/paddleinference.tar.gz",
        ],
    )
