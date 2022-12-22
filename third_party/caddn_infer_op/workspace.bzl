"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "caddn_infer_op",
        sha256 = "8cf2f7444de837b80a5a23783dc58f8e372a5eb61f76cbbc543f0c036ecc73ce",
        strip_prefix = "caddn_infer_op",
        urls = [
            "https://apollo-system.bj.bcebos.com/archive/v8.0_bev/caddn_infer_op.tar.gz",
        ],
    )
