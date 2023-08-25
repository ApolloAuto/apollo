load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "msg_adapter_files",
    srcs = glob([
        "dag/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_msg_adapter",
    srcs = [
        "common/msg_adapter_gflags.cc",
    ],
    hdrs = [
        "common/msg_adapter_gflags.h",
        "convert/common.h",
        "convert/convert.h",
        "msg_converter.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_component(
    name = "libmsg_adapter_component.so",
    srcs = ["msg_adapter_component.cc"],
    hdrs = ["msg_adapter_component.h"],
    deps = [
        "//cyber",
        ":apollo_perception_msg_adapter"
    ],
)

apollo_package()

cpplint()
