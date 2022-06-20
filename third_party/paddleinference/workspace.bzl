"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

#def repo():
#    native.new_local_repository(
#        name = "paddleinference",
#        build_file = clean_dep("//third_party/paddleinference:paddleinference.BUILD"),
#        path = "/usr/local/paddleinference",
#    )



def repo():
    http_archive(
        name = "paddleinference",
        sha256 = "a0aec037568208dd4872c3658b497b1d3087e4574a3e99f826798419878ada63",
        strip_prefix = "paddleinference",
        urls = [
            "file:///apollo/paddle_inference_workspace/paddleinference.tar.gz",
        ],
    )
