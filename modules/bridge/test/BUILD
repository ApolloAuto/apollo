load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_binary")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "bridge_receiver_test",
    srcs = ["bridge_receiver_test.cc"],
    deps = [
        "//cyber",
        "//modules/bridge:apollo_udp_bridge",
    ],
)

apollo_cc_binary(
    name = "bridge_sender_test",
    srcs = ["bridge_sender_test.cc"],
    deps = [
        "//cyber",
        "//modules/bridge:apollo_udp_bridge",
    ],
)

apollo_package()
cpplint()
