"""Loads the nlohmann_json library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "com_github_nlohmann_json",
        sha256 = "7d0edf65f2ac7390af5e5a0b323b31202a6c11d744a74b588dc30f5a8c9865ba",
        strip_prefix = "json-3.8.0",
        build_file = clean_dep("//third_party/nlohmann_json:json.BUILD"),
        urls = [
            "https://apollo-system.cdn.bcebos.com/archive/6.0/v3.8.0.tar.gz",
            "https://github.com/nlohmann/json/archive/v3.8.0.tar.gz",
        ],
    )
