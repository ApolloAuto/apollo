load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "bridge/bin",
    targets = [
        ":bridge_receiver_test",
        ":bridge_sender_test",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "bridge_receiver_test",
    srcs = ["bridge_receiver_test.cc"],
    deps = [
        "//cyber",
        "//modules/bridge:udp_bridge",
        "//modules/bridge/common:udp_listener",
    ],
)

cc_binary(
    name = "bridge_sender_test",
    srcs = ["bridge_sender_test.cc"],
    deps = [
        "//cyber",
        "//modules/bridge:udp_bridge",
    ],
)

cpplint()
