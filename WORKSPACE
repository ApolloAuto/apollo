#######################################APOLLO#######################################
workspace(name = "apollo")

load("//tools:workspace.bzl", "apollo_repositories")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "rules_foreign_cc",
    sha256 = "6041f1374ff32ba711564374ad8e007aef77f71561a7ce784123b9b4b88614fc",
    strip_prefix = "rules_foreign_cc-0.8.0",
    urls = [
        "https://apollo-system.bj.bcebos.com/archive/6.0/rules_foreign_cc-0.8.0.tar.gz",
        "https://github.com/bazelbuild/rules_foreign_cc/archive/0.8.0.tar.gz",
    ],
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()

apollo_repositories()

http_archive(
    name = "rules_cc",
    urls = [
        "https://apollo-system.cdn.bcebos.com/archive/8.0/rules_cc-0.0.1.tar.gz",
        "https://github.com/bazelbuild/rules_cc/releases/download/0.0.1/rules_cc-0.0.1.tar.gz",
    ],
    sha256 = "4dccbfd22c0def164c8f47458bd50e0c7148f3d92002cdb459c2a96a68498241",
    patches = ["//tools/package:rules_cc.patch"],
)

http_archive(
    name = "bazel_skylib",
    sha256 = "1c531376ac7e5a180e0237938a2536de0c54d93f5c278634818e0efc952dd56c",
    urls = [
        "https://apollo-system.cdn.bcebos.com/archive/6.0/bazel-skylib-1.0.3.tar.gz",
        "https://github.com/bazelbuild/bazel-skylib/releases/download/1.0.3/bazel-skylib-1.0.3.tar.gz",
    ],
)

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

load("@bazel_skylib//lib:versions.bzl", "versions")

versions.check(minimum_bazel_version = "3.7.0")

http_archive(
    name = "rules_proto",
    sha256 = "602e7161d9195e50246177e7c55b2f39950a9cf7366f74ed5f22fd45750cd208",
    strip_prefix = "rules_proto-97d8af4dc474595af3900dd85cb3a29ad28cc313",
    urls = [
        "https://apollo-system.cdn.bcebos.com/archive/6.0/97d8af4dc474595af3900dd85cb3a29ad28cc313.tar.gz",
        "https://github.com/bazelbuild/rules_proto/archive/97d8af4dc474595af3900dd85cb3a29ad28cc313.tar.gz",
    ],
)

load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")

rules_proto_dependencies()

rules_proto_toolchains()

http_archive(
    name = "rules_python",
    sha256 = "b6d46438523a3ec0f3cead544190ee13223a52f6a6765a29eae7b7cc24cc83a0",
    urls = [
        "https://apollo-system.cdn.bcebos.com/archive/6.0/rules_python-0.1.0.tar.gz",
        "https://github.com/bazelbuild/rules_python/releases/download/0.1.0/rules_python-0.1.0.tar.gz",
    ],
)

# load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")
# grpc
http_archive(
    name = "com_github_grpc_grpc",
    sha256 = "2378b608557a4331c6a6a97f89a9257aee2f8e56a095ce6619eea62e288fcfbe",
    patches = ["//third_party/absl:grpc.patch"],
    strip_prefix = "grpc-1.30.0",
    urls = [
        "https://apollo-system.cdn.bcebos.com/archive/8.0/v1.30.0-apollo.tar.gz",
    ],
)
http_archive(
    name = "zlib",
    build_file = "@com_google_protobuf//:third_party/zlib.BUILD",
    sha256 = "629380c90a77b964d896ed37163f5c3a34f6e6d897311f1df2a7016355c45eff",
    strip_prefix = "zlib-1.2.11",
    urls = [
        "https://apollo-system.cdn.bcebos.com/archive/6.0/zlib-v1.2.11.tar.gz",
        "https://github.com/madler/zlib/archive/v1.2.11.tar.gz",
    ],
)

load("@com_github_grpc_grpc//bazel:grpc_deps.bzl", "grpc_deps")

grpc_deps()

load("@com_github_grpc_grpc//bazel:grpc_extra_deps.bzl", "grpc_extra_deps")

grpc_extra_deps()
#######################################APOLLO#######################################