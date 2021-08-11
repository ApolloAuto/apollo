"""Loads the yaml-cpp library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "com_github_jbeder_yaml_cpp",
        build_file = clean_dep("//third_party/yaml_cpp:yaml_cpp.BUILD"),
        sha256 = "77ea1b90b3718aa0c324207cb29418f5bced2354c2e483a9523d98c3460af1ed",
        strip_prefix = "yaml-cpp-yaml-cpp-0.6.3",
        urls = [
            "https://apollo-system.cdn.bcebos.com/archive/6.0/yaml-cpp-0.6.3.tar.gz",
            "https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.3.tar.gz",
        ],
    )
