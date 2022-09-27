"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "centerpoint_infer_op",
        sha256 = "6cdf00df32f43d7af671ac0094d33535932163125091a670216ff32f8063fe4b",
        strip_prefix = "centerpoint_infer_op",
        urls = [
            "https://apollo-system.bj.bcebos.com/archive/8.0/centerpoint_infer_op.tar.gz",
        ],
    )
