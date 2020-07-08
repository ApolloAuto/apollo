"""Loads the cpplint library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "cpplint",
        build_file = clean_dep("//third_party/cpplint:cpplint.BUILD"),
        sha256 = "96db293564624543a2fd3b1a0d23f663b8054c79853a5918523655721a9f6b53",
        strip_prefix = "cpplint-1.4.5",
        urls = ["https://github.com/cpplint/cpplint/archive/1.4.5.tar.gz"],
    )
