load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    targets = [
      ":cyber_bridge",
    ],
    runtime_dest = "modules/contrib/cyber_bridge",
    deps = [
        "//cyber:install",
    ],
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
