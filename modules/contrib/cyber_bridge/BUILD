load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_bridge",
    srcs = ["bridge.cc"],
    linkopts = [
        "-lboost_system",
    ],
    deps = [
        ":bridge_server",
        "//external:gflags",
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
    ],
)

cc_library(
    name = "bridge_clients",
    srcs = ["clients.cc"],
    hdrs = ["clients.h"],
    deps = [
        ":bridge_client_header",
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
    ],
)

cpplint()
