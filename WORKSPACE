
#######################################APOLLO#######################################
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
http_archive(
    name = "build_bazel_rules_apple",
    sha256 = "0052d452af7742c8f3a4e0929763388a66403de363775db7e90adecb2ba4944b",
    urls = [
        "https://apollo-system.cdn.bcebos.com/archive/8.0/rules_apple.0.31.3.tar.gz",
        "https://github.com/bazelbuild/rules_apple/releases/download/0.31.3/rules_apple.0.31.3.tar.gz",
    ],
)
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
http_archive(
    name = "rules_cc",
    urls = ["https://apollo-system.cdn.bcebos.com/archive/8.0/rules_cc-0.0.1.tar.gz", "https://github.com/bazelbuild/rules_cc/releases/download/0.0.1/rules_cc-0.0.1.tar.gz"],
    sha256 = "4dccbfd22c0def164c8f47458bd50e0c7148f3d92002cdb459c2a96a68498241",
    patches = ["//tools/package:rules_cc.patch"],
)
load("//dev/bazel:deps.bzl", "init_deps")
init_deps()
load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
bazel_skylib_workspace()
load("@com_github_grpc_grpc//bazel:grpc_deps.bzl", "grpc_deps")
grpc_deps()
load("@com_github_grpc_grpc//bazel:grpc_extra_deps.bzl", "grpc_extra_deps")
grpc_extra_deps()
load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")
rules_proto_dependencies()
rules_proto_toolchains()
#######################################APOLLO#######################################

http_archive(
        name = "com_github_openssl_openssl",
        build_file = "//third_party/openssl:openssl.BUILD",
        sha256 = "0686897afd3a08223760db73d8034550401b53ffc545798d7ca476564f80315e",
        strip_prefix = "openssl-OpenSSL_1_1_1q",
        urls = [
            "https://apollo-system.bj.bcebos.com/archive/6.0/OpenSSL_1_1_1q.tar.gz",
            "https://github.com/openssl/openssl/archive/refs/tags/OpenSSL_1_1_1q.tar.gz",
        ],
)