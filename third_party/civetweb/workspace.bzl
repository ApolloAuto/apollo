"""Loads the civetweb library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "civetweb",
        build_file = clean_dep("//third_party/civetweb:civetweb.BUILD"),
        sha256 = "de7d5e7a2d9551d325898c71e41d437d5f7b51e754b242af897f7be96e713a42",
        strip_prefix = "civetweb-1.11",
        urls = [
            "https://apollo-system.cdn.bcebos.com/archive/6.0/v1.11.tar.gz",
            "https://github.com/civetweb/civetweb/archive/v1.11.tar.gz",
        ],
    )
