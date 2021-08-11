"""Loads the absl library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "com_google_absl",
        sha256 = "f41868f7a938605c92936230081175d1eae87f6ea2c248f41077c8f88316f111",
        strip_prefix = "abseil-cpp-20200225.2",
        urls = [
            "https://apollo-system.cdn.bcebos.com/archive/6.0/20200225.2.tar.gz",
            "https://github.com/abseil/abseil-cpp/archive/20200225.2.tar.gz",
        ],
    )
