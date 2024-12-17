"""Loads the protobuf library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "com_google_protobuf",
        # sha256 = "930c2c3b5ecc6c9c12615cf5ad93f1cd6e12d0aba862b572e076259970ac3a53",
        # patches = [clean_dep("//third_party/protobuf:protobuf.patch")],
        # patch_args = ["-p1"],
        # strip_prefix = "protobuf-3.21.12",
        sha256 = "d0f5f605d0d656007ce6c8b5a82df3037e1d8fe8b121ed42e536f569dec16113",
        patches = [clean_dep("//third_party/protobuf:protobuf_3.14.0.patch")],
        strip_prefix = "protobuf-3.14.0",
        urls = [
            "https://github.com/protocolbuffers/protobuf/archive/v3.14.0.tar.gz",
        ],
    )
