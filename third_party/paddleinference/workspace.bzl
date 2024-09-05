# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "paddleinference-x86_64",
        sha256 = "7498df1f9bbaf5580c289a67920eea1a975311764c4b12a62c93b33a081e7520",
        strip_prefix = "paddleinference",
        urls = ["https://apollo-pkg-beta.cdn.bcebos.com/archive/paddleinference-cu118-x86.tar.gz"],
    )

    http_archive(
        name = "paddleinference-aarch64",
        sha256 = "ac5f124650e61d8d4b3552cf070258bc2464293bc31d7416ee99e9ba9693e3ee",
        strip_prefix = "paddleinference",
        urls = ["https://apollo-pkg-beta.bj.bcebos.com/archive/paddleinference-linux-aarch64-2.0.0.tar.gz"],
    )
