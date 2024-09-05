"""Loads the gtest library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def repo():
    # googletest (GTest and GMock)
    http_archive(
        name = "com_google_googletest",
        # sha256 = "9dc9157a9a1551ec7a7e43daea9a694a0bb5fb8bec81235d8a1e6ef64c716dcb",
        sha256 = "ad7fdba11ea011c1d925b3289cf4af2c66a352e18d4c7264392fead75e919363",
        # strip_prefix = "googletest-release-1.10.0",
        strip_prefix = "googletest-1.13.0",
        urls = [
            # "https://apollo-system.cdn.bcebos.com/archive/6.0/release-1.10.0.tar.gz",
            # "https://github.com/google/googletest/archive/release-1.10.0.tar.gz",
            "https://github.com/google/googletest/archive/v1.13.0.tar.gz",
        ],
    )
