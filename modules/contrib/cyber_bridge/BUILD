load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_bridge",
    srcs = [
        "bridge.cc",
        "node.cc",
        "node.h",
        "client.cc",
        "client.h",
        "clients.cc",
        "clients.h",
        "server.cc",
        "server.h",
    ],
    linkopts = [
        "-lboost_system",
    ],
    deps = [
        "//cyber",
        "//external:gflags",
    ],
)

cpplint()
