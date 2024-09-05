load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_gpu")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
        "data/**",
    ]),
)

apollo_plugin(
    name = "liblocalization_estimate.so",
    srcs = ["localization_estimate.cc"],
    hdrs = ["localization_estimate.h"],
    description = ":localization_estimate.xml",
    deps = [
        "//cyber",
        "//cyber/ros_bridge:converter_base",
        "//cyber/proto:simple_proto",
        "//modules/common_msgs/localization_msgs:localization_proto",
        "//modules/common_msgs/transform_msgs:transform_proto",
        "//cyber/ros_bridge/proto:converter_conf_proto",
        "@ros"
    ],
)

apollo_package()

cpplint()
