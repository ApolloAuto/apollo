"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "caddn_infer_op",
        sha256 = "bcb203466d949415a8e88c9df7783243240db5d091b60cd977083b254954a2c0",
        strip_prefix = "caddn_infer_op",
        urls = [
            "https://apollo-system.bj.bcebos.com/archive/v8.0_bev/caddn_infer_op.tar.gz",
        ],
    )
