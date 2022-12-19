load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "pad_terminal",
    srcs = ["pad_terminal.cc"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/planning/common:planning_gflags",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "inference_demo",
    srcs = ["inference_demo.cc"],
    deps = [
        #"//third_party/libtorch",
        "@com_github_gflags_gflags//:gflags",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

cpplint()
