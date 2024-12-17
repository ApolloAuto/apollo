"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "centerpoint_infer_op-x86_64",
        sha256 = "038470fc2e741ebc43aefe365fc23400bc162c1b4cbb74d8c8019f84f2498190",
        strip_prefix = "centerpoint_infer_op",
        urls = ["https://apollo-pkg-beta.bj.bcebos.com/archive/centerpoint_infer_op_cu118.tar.gz"],
    )

    http_archive(
        name = "centerpoint_infer_op-aarch64",
        sha256 = "5d761076e139ef9e973541d7c0d2196cf789e55e14d9b9df0574bbea7caf184c",
        strip_prefix = "centerpoint_infer_op",
        urls = ["https://apollo-pkg-beta.bj.bcebos.com/archive/centerpoint_infer_op-linux-aarch64-2.0.0.tar.gz"],
    )
