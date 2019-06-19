load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "bridge_receiver_test",
    srcs = ["bridge_receiver_test.cc"],
    deps = [
        "//cyber",
        "//cyber/examples/proto:examples_cc_proto",
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
