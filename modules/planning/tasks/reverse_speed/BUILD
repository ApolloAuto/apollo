load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libreverse_speed.so",
    srcs = ["reverse_speed.cc"],
    hdrs = ["reverse_speed.h"],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util:common_util",
        "//modules/common_msgs/planning_msgs:decision_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/reverse_speed/proto:reverse_speed_cc_proto",
    ],
)

apollo_package(enable_source=False)

cpplint()
