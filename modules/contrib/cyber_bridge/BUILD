load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "cyber-bridge",
    data = [
        ":cyberfile.xml",
        ":cyber-bridge.BUILD",
    ],
    targets = [
      ":cyber_bridge",
    ],
    runtime_dest = "cyber-bridge/bin",
    deps = [
        "//cyber:install",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_cyber-bridge_src",
        ":install_cyber-bridge_hdrs"
    ],
)

install_src_files(
    name = "install_cyber-bridge_src",
    src_dir = ["."],
    dest = "cyber-bridge/src",
    filter = "*",
)

install_src_files(
    name = "install_cyber-bridge_hdrs",
    src_dir = ["."],
    dest = "cyber-bridge/include",
    filter = "*.h",
)

cc_binary(
    name = "cyber_bridge",
    srcs = ["bridge.cc"],
    deps = [
        ":bridge_server",
        "@boost",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "bridge_server",
    srcs = ["server.cc"],
    hdrs = ["server.h"],
    deps = [
        ":bridge_client",
    ],
)

cc_library(
    name = "bridge_client",
    srcs = ["client.cc"],
    deps = [
        ":bridge_clients",
        ":bridge_node",
        "@boost",
    ],
)

cc_library(
    name = "bridge_clients",
    srcs = ["clients.cc"],
    hdrs = ["clients.h"],
    deps = [
        ":bridge_client_header",
        "@boost",
    ],
)

cc_library(
    name = "bridge_node",
    srcs = ["node.cc"],
    hdrs = ["node.h"],
    deps = [
        ":bridge_client_header",
    ],
)

cc_library(
    name = "bridge_client_header",
    hdrs = ["client.h"],
    deps = [
        "//cyber",
        "@boost",
    ],
)

cpplint()
