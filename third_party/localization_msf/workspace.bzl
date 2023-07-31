"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "localization_msf",
        sha256 = "58e11d580060ad9b75d62254eb4f943c510acf5e1eb88868797aa7d77d32299b",
        strip_prefix = "localization_msf",
        urls = ["https://apollo-pkg-beta.bj.bcebos.com/archive/localization_msf-linux-any-1.0.0.tar.gz"],
    )