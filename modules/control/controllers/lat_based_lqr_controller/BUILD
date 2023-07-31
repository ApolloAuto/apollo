load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']


cyber_plugin_description(
    name = "plugin_lat_controller_description",
    description = ":plugins.xml",
    plugin = ":liblat_controller.so",
)

cc_binary(
    name = "liblat_controller.so",
    linkshared = True,
    linkstatic = True,
    copts = ["-DMODULE_NAME=\\\"control\\\""],
    deps = [":lat_controller_lib"],
)

cc_library(
    name = "lat_controller_lib",
    srcs = ["lat_controller.cc"],
    hdrs = ["lat_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/controllers/control_task_base:control_task",
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/controllers/control_task_base/common:interpolation_1d",
        "//modules/control/controllers/control_task_base/common:leadlag_controller",
        "//modules/control/controllers/control_task_base/common:mrac_controller",
        "//modules/control/controllers/control_task_base/common:trajectory_analyzer",
        "//modules/control/controllers/control_task_base/proto:calibration_table_cc_proto",
        "//modules/control/controllers/lat_based_lqr_controller/proto:lat_based_lqr_controller_conf_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/control/controllers/control_task_base/proto:gain_scheduler_conf_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "//modules/control/controllers/control_task_base/proto:mrac_conf_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "@com_google_absl//:absl",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "lat_controller_test",
    size = "small",
    srcs = ["lat_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["lateral_controller_test"],
    deps = [
        ":lat_controller_lib",
        "//cyber",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/controllers/control_task_base:control_task",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "lateral_controller_test",
    srcs = glob([
        "lateral_controller_test/**",
    ]) + glob(["conf/*"]),
)

apollo_package()
cpplint()