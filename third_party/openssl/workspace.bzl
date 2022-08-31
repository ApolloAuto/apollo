"""Loads the openssl library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "com_github_openssl_openssl",
        build_file = clean_dep("//third_party/openssl:openssl.BUILD"),
        sha256 = "0686897afd3a08223760db73d8034550401b53ffc545798d7ca476564f80315e",
        strip_prefix = "openssl-OpenSSL_1_1_1q",
        urls = [
            "https://apollo-system.bj.bcebos.com/archive/6.0/OpenSSL_1_1_1q.tar.gz",
            "https://github.com/openssl/openssl/archive/refs/tags/OpenSSL_1_1_1q.tar.gz",
        ],
    )
