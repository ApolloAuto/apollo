"""Loads the protobuf library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    http_archive(
        name = "com_google_protobuf",
        sha256 = "9b4ee22c250fe31b16f1a24d61467e40780a3fbb9b91c3b65be2a376ed913a1a",
        strip_prefix = "protobuf-3.13.0",
        urls = ["https://github.com/protocolbuffers/protobuf/archive/v3.13.0.tar.gz"],
    )
