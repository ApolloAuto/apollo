load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

GNSS_COPTS = ['-DMODULE_NAME=\\"gnss\\"']

apollo_cc_library(
    name = "apollo_drivers_gnss",
    srcs = glob([
        "parser/*.cc",
        "parser/**/*.cc",
        "stream/*.cc",
    ]),
    hdrs = glob([
        "parser/*.h",
        "parser/**/*.h",
        "stream/*.h",
        "util/*.h",
    ]),
    copts = GNSS_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_raw_observation_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "//modules/drivers/gnss/proto:config_cc_proto",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/transform:apollo_transform",
        "//third_party/rtklib",

        "@com_google_absl//:absl",
        "@eigen",
        "@proj",
    ],
)

apollo_cc_binary(
    name = "parser_cli",
    srcs = ["test/parser_cli.cc"],
    deps = [
        "//cyber",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        ":apollo_drivers_gnss",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_component(
    name = "libgnss_component.so",
    srcs = ["gnss_component.cc",],
    hdrs = ["gnss_component.h",],
    deps = [":apollo_drivers_gnss"]
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_package()

cpplint()