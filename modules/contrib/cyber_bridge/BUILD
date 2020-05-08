load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_bridge",
    srcs = [
        "bridge.cc",
    ],
    linkopts = [
        "-lboost_system",
    ],
    deps = [
        ":bridge_client",
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
    srcs = [
        "client.cc",
        "clients.cc",
        "node.cc",
    ],
    hdrs = [
        "client.h",
        "clients.h",
        "node.h",
    ],    
    deps = [
        "//cyber",
    ],
)

cpplint()
