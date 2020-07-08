"""Loads the protobuf library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "com_google_protobuf",
        sha256 = "71030a04aedf9f612d2991c1c552317038c3c5a2b578ac4745267a45e7037c29",
        strip_prefix = "protobuf-3.12.3",
        urls = ["https://github.com/protocolbuffers/protobuf/archive/v3.12.3.tar.gz"],
    )
