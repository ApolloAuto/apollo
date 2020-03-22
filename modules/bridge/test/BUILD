load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
