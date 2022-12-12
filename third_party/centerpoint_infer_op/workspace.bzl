"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "centerpoint_infer_op",
        sha256 = "b39430eafad6b8b9b7e216840845b01c24b66406fb94ce06df721c6a3d738054",
        strip_prefix = "centerpoint_infer_op",
        urls = [
            "https://apollo-system.bj.bcebos.com/archive/v8.0_bev/centerpoint_infer_op.tar.gz",
        ],
    )
