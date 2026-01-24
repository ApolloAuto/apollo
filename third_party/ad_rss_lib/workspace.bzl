"""Loads the ad_rss_lib library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "ad_rss_lib",
        build_file = clean_dep("//third_party/ad_rss_lib:ad_rss_lib.BUILD"),
        sha256 = "10c161733a06053f79120f389d2d28208c927eb65759799fb8d7142666b61b9f",
        strip_prefix = "ad-rss-lib-1.1.0",
        urls = [
            "https://apollo-system.cdn.bcebos.com/archive/6.0/v1.1.0.tar.gz",
            "https://github.com/intel/ad-rss-lib/archive/v1.1.0.tar.gz",
        ],
    )
