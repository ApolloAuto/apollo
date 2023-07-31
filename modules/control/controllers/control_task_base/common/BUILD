load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_binary(
    name = "libcommon.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":dependency_injector",
        ":hysteresis_filter",
        ":interpolation_1d",
        ":interpolation_2d",
        ":leadlag_controller",
        ":mrac_controller",
        ":pid_controller",
        ":pid_BC_controller",
        ":pid_IC_controller",
        ":trajectory_analyzer",
    ],
)

cc_library(
    name = "common",
    srcs = ["libcommon.so"],
    hdrs = glob(["*.h"]),
    deps = [
        ":hysteresis_filter",
        "//cyber",
        "//modules/control/controllers/control_task_base/proto:leadlag_conf_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/control/controllers/control_task_base/proto:mrac_conf_cc_proto",
        "//modules/control/controllers/control_task_base/proto:pid_conf_cc_proto",
        "@com_google_absl//:absl",
        "@eigen",
        ":dependency_injector",
        ":interpolation_1d",
        ":interpolation_2d",
        ":leadlag_controller",
        ":mrac_controller",
        ":pid_controller",
        ":pid_BC_controller",
        ":pid_IC_controller",
        ":trajectory_analyzer",
    ],
    visibility = ["//visibility:public"],
    alwayslink = True,
)

cc_library(
    name = "dependency_injector",
    hdrs = ["dependency_injector.h"],
    deps = [
        "//modules/common/vehicle_state:vehicle_state_provider",
    ],
    alwayslink = True,
)

cc_library(
    name = "hysteresis_filter",
    srcs = ["libhysteresis_filter.so"],
    hdrs = ["hysteresis_filter.h"],
    copts = CONTROL_COPTS,
    alwayslink = True,
)

cc_binary(
    name = "libhysteresis_filter.so",
    srcs = [
        "hysteresis_filter.cc",
        "hysteresis_filter.h",
    ],
    linkshared = True,
    linkstatic = True,
    #copts = CONTROL_COPTS,
)

cc_library(
    name = "interpolation_1d",
    srcs = ["interpolation_1d.cc"],
    hdrs = ["interpolation_1d.h"],
    alwayslink = True,
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "interpolation_2d",
    srcs = ["interpolation_2d.cc"],
    hdrs = ["interpolation_2d.h"],
    alwayslink = True,
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
    ],
)

cc_library(
    name = "leadlag_controller",
    srcs = ["leadlag_controller.cc"],
    hdrs = ["leadlag_controller.h"],
    alwayslink = True,
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/controllers/control_task_base/proto:leadlag_conf_cc_proto",
    ],
)

cc_library(
    name = "mrac_controller",
    srcs = ["mrac_controller.cc"],
    hdrs = ["mrac_controller.h"],
    alwayslink = True,
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/control/controllers/control_task_base/proto:mrac_conf_cc_proto",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

cc_library(
    name = "pid_controller",
    srcs = ["pid_controller.cc"],
    hdrs = ["pid_controller.h"],
    alwayslink = True,
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/controllers/control_task_base/proto:pid_conf_cc_proto",
    ],
)

cc_library(
    name = "pid_BC_controller",
    srcs = ["pid_BC_controller.cc"],
    hdrs = ["pid_BC_controller.h"],
    alwayslink = True,
    copts = CONTROL_COPTS,
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/common/math",
        "//modules/control/controllers/control_task_base/proto:pid_conf_cc_proto",
    ],
)

cc_library(
    name = "pid_IC_controller",
    srcs = ["pid_IC_controller.cc"],
    hdrs = ["pid_IC_controller.h"],
    alwayslink = True,
    copts = CONTROL_COPTS,
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/common/math",
        "//modules/control/controllers/control_task_base/proto:pid_conf_cc_proto",
    ],
)

cc_library(
    name = "trajectory_analyzer",
    srcs = ["trajectory_analyzer.cc"],
    hdrs = ["trajectory_analyzer.h"],
    alwayslink = True,
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/control_component/common:control_gflags",
        "//cyber",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@eigen",
    ],
)

cc_test(
    name = "hysteresis_filter_test",
    size = "small",
    srcs = ["hysteresis_filter_test.cc"],
    deps = [
        ":hysteresis_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "interpolation_1d_test",
    size = "small",
    srcs = ["interpolation_1d_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":interpolation_1d",
        "//cyber",
        "//modules/control/controllers/lat_based_lqr_controller/proto:lat_based_lqr_controller_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "interpolation_2d_test",
    size = "small",
    srcs = ["interpolation_2d_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":interpolation_2d",
        "//cyber",
        "//modules/control/controllers/control_task_base/proto:calibration_table_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "leadlag_controller_test",
    size = "small",
    srcs = ["leadlag_controller_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":leadlag_controller",
        "//cyber",
        "//modules/control/controllers/lon_based_pid_controller/proto:lon_based_pid_controller_conf_cc_proto",
        "//modules/control/controllers/control_task_base/proto:leadlag_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "mrac_controller_test",
    size = "small",
    srcs = ["mrac_controller_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":mrac_controller",
        "//cyber",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/control/controllers/lat_based_lqr_controller/proto:lat_based_lqr_controller_conf_cc_proto",
        "//modules/control/controllers/control_task_base/proto:mrac_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pid_controller_test",
    size = "small",
    srcs = ["pid_controller_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/control/controllers/lon_based_pid_controller/proto:lon_based_pid_controller_conf_cc_proto",
        "//modules/control/controllers/control_task_base/proto:pid_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pid_BC_controller_test",
    size = "small",
    srcs = ["pid_BC_controller_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":pid_BC_controller",
        "//cyber",
        "//modules/control/controllers/lon_based_pid_controller/proto:lon_based_pid_controller_conf_cc_proto",
        "//modules/control/controllers/control_task_base/proto:pid_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pid_IC_controller_test",
    size = "small",
    srcs = ["pid_IC_controller_test.cc"],
    data = ["//modules/control/control_component:test_data"],
    deps = [
        ":pid_IC_controller",
        "//cyber",
        "//modules/control/controllers/lon_based_pid_controller/proto:lon_based_pid_controller_conf_cc_proto",
        "//modules/control/controllers/control_task_base/proto:pid_conf_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "trajectory_analyzer_test",
    size = "small",
    srcs = ["trajectory_analyzer_test.cc"],
    deps = [
        ":trajectory_analyzer",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
