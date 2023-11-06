"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "paddleinference-x86_64",
        sha256 = "055492fd2c43210640382b8ff2a8de16a6d53e77cc2e94cc9c8d3c6c830b9e2e",
        strip_prefix = "paddleinference",
        urls = ["https://apollo-pkg-beta.bj.bcebos.com/archive/paddleinferencev-1.0.0.tar.gz"],
    )

    http_archive(
        name = "paddleinference-aarch64",
        sha256 = "048d1d7799ffdd7bd8876e33bc68f28c3af911ff923c10b362340bd83ded04b3",
        strip_prefix = "paddleinference",
        urls = ["https://apollo-pkg-beta.bj.bcebos.com/archive/paddleinference-linux-aarch64-1.0.0.tar.gz"],
    )
