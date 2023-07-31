"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "paddleinference-x86_64",
        sha256 = "d79ae353d0181035eb6c750d1e9e8cf50148e3c8648dd1fa084e50af64380b1c",
        strip_prefix = "paddleinference",
        urls = ["https://apollo-system.bj.bcebos.com/archive/v8.0_bev/paddleinferencev-1.0.0.tar.gz"],
    )

    http_archive(
        name = "paddleinference-aarch64",
        sha256 = "048d1d7799ffdd7bd8876e33bc68f28c3af911ff923c10b362340bd83ded04b3",
        strip_prefix = "paddleinference",
        urls = ["https://apollo-pkg-beta.bj.bcebos.com/archive/paddleinference-linux-aarch64-1.0.0.tar.gz"],
    )
