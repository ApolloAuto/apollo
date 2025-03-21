"""Loads the sse2neon library"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    # sse2neon
    http_archive(
        name = "sse2neon",
        build_file = clean_dep("//third_party/sse2neon:sse2neon.BUILD"),
        strip_prefix = "sse2neon-1.6.0",
        urls = [
            "https://github.com/DLTcollab/sse2neon/archive/refs/tags/v1.6.0.tar.gz",
        ],
    )
