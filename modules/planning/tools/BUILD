load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "pad_terminal",
    srcs = ["pad_terminal.cc"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/planning/proto:planning_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "inference_demo",
    srcs = ["inference_demo.cc"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
        "//third_party:libtorch",
    ],
)

cpplint()
