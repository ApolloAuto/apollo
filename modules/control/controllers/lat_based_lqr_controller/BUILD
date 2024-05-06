load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_test", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_plugin(
    name = "liblat_controller.so",
    srcs = ["lat_controller.cc"],
    hdrs = ["lat_controller.h"],
    description = ":plugins.xml",
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/control_component/controller_task_base:control_task",
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base/common:interpolation_1d",
        "//modules/control/control_component/controller_task_base/common:leadlag_controller",
        "//modules/control/control_component/controller_task_base/common:mrac_controller",
        "//modules/control/control_component/controller_task_base/common:trajectory_analyzer",
        "//modules/control/control_component/proto:calibration_table_cc_proto",
        "//modules/control/controllers/lat_based_lqr_controller/proto:lat_based_lqr_controller_conf_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/control/control_component/proto:gain_scheduler_conf_cc_proto",
        "//modules/common_msgs/control_msgs:input_debug_cc_proto",
        "//modules/control/control_component/proto:mrac_conf_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "lat_controller_test",
    size = "small",
    srcs = ["lat_controller_test.cc"],
    copts = ["-fno-access-control"],
    data = ["lateral_controller_test"],
    deps = [
        ":lat_controller_lib",
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/control_component/common:control_gflags",
        "//modules/control/control_component/controller_task_base:control_task",
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