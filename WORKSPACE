workspace(name = "apollo")

load("//tools:workspace.bzl", "apollo_repositories")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

apollo_repositories()

http_archive(
    name = "bazel_skylib",
    sha256 = "1dde365491125a3db70731e25658dfdd3bc5dbdfd11b840b3e987ecf043c7ca0",
    urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/0.9.0/bazel_skylib-0.9.0.tar.gz"],
)

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

http_archive(
    name = "rules_proto",
    sha256 = "602e7161d9195e50246177e7c55b2f39950a9cf7366f74ed5f22fd45750cd208",
    strip_prefix = "rules_proto-97d8af4dc474595af3900dd85cb3a29ad28cc313",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_proto/archive/97d8af4dc474595af3900dd85cb3a29ad28cc313.tar.gz",
        "https://github.com/bazelbuild/rules_proto/archive/97d8af4dc474595af3900dd85cb3a29ad28cc313.tar.gz",
    ],
)

load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")

rules_proto_dependencies()

rules_proto_toolchains()

http_archive(
    name = "rules_python",
    sha256 = "b5668cde8bb6e3515057ef465a35ad712214962f0b3a314e551204266c7be90c",
    strip_prefix = "rules_python-0.0.2",
    url = "https://github.com/bazelbuild/rules_python/releases/download/0.0.2/rules_python-0.0.2.tar.gz",
)

load("@rules_python//python:repositories.bzl", "py_repositories")

py_repositories()

# Only needed if using the packaging rules.
# load("@rules_python//python:pip.bzl", "pip_repositories")
# pip_repositories()

# load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")
# See https://github.com/bazelbuild/bazel/issues/11406
# maybe(
#    http_archive,
#    name = "boringssl",
#    sha256 = "fb236ae74676dba515e1230aef4cc69ab265af72fc08784a6755a319dd013ca6",
#    urls = ["https://apollo-platform-system.bj.bcebos.com/archive/6.0/boringssl-83da28a68f32023fd3b95a8ae94991a07b1f6c62.tar.gz"],
# )
# grpc
http_archive(
    name = "com_github_grpc_grpc",
    sha256 = "419dba362eaf8f1d36849ceee17c3e2ff8ff12ac666b42d3ff02a164ebe090e9",
    strip_prefix = "grpc-1.30.0",
    urls = ["https://github.com/grpc/grpc/archive/v1.30.0.tar.gz"],
)

load("@com_github_grpc_grpc//bazel:grpc_deps.bzl", "grpc_deps")

grpc_deps()

load("@com_github_grpc_grpc//bazel:grpc_extra_deps.bzl", "grpc_extra_deps")

grpc_extra_deps()

#new_local_repository(
#    name = "opengl",
#    build_file = "third_party/opengl.BUILD",
#    path = "/usr/include",
#)
#new_local_repository(
#    name = "glfw",
#    build_file = "third_party/glfw.BUILD",
#    path = "/usr/include",
#)
# Caffe
# new_local_repository(
#    name = "caffe",
#    build_file = "third_party/caffe.BUILD",
#    path = "/opt/apollo/pkgs/caffe/include",
# )

#new_local_repository(
#    name = "mkldnn",
#    build_file = "third_party/mkldnn.BUILD",
#    path = "/usr/local/apollo/local_third_party/mkldnn",
#)
#
# mklml
# new_local_repository(
#    name = "mklml",
#    build_file = "third_party/mklml.BUILD",
#    path = "/usr/local/apollo/local_third_party/mklml",
# )
