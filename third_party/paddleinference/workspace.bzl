"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "paddleinference",
        sha256 = "b205e321110cdd7e819251e0e8bd8849731fb79bdece552ffc87d73fce3b2fb3",
        strip_prefix = "paddleinference",
        urls = [
            "https://apollo-system.bj.bcebos.com/archive/v8.0_bev/paddleinference.tar.gz",
        ],
    )
