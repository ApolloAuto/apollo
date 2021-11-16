"""Loads the benchmark library"""

#@unused
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # benchmark
    # native.new_local_repository(
    #    name = "com_google_benchmark",
    #    build_file = clean_dep("//third_party/benchmark:benchmark.BUILD"),
    #    path = "/opt/apollo/sysroot/include",
    # )
    http_archive(
        name = "com_google_benchmark",
        sha256 = "23082937d1663a53b90cb5b61df4bcc312f6dee7018da78ba00dd6bd669dfef2",
        strip_prefix = "benchmark-1.5.1",
        urls = [
            "https://apollo-system.cdn.bcebos.com/archive/6.0/v1.5.1.tar.gz",
            "https://github.com/google/benchmark/archive/v1.5.1.tar.gz",
        ],
    )
