load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "pad_terminal",
    srcs = ["pad_terminal.cc"],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/planning/proto:planning_proto",
    ],
)

cpplint()
