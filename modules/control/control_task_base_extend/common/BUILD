load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_cc_library(
    name = "controllers_control_task_base_extend_common",
    hdrs = glob(["*.h"]),
    visibility = ["//visibility:public"],
    deps = [
        ":control_checker",
        ":exponential_smoothing",
        ":interpolation_plus_1d",
        ":leadlag_plus_controller",
        ":math_utils",
        ":pid_plus_controller",
        ":trajectory_analyzer_extend",
    ],
)

apollo_cc_library(
    name = "control_checker",
    srcs = ["control_checker.cc"],
    hdrs = ["control_checker.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/control/control_component/proto:check_status_proto",
        "//modules/control/control_task_base_extend/proto:safety_check_conf_proto",

    ],
)

apollo_cc_library(
    name = "exponential_smoothing",
    srcs = ["exponential_smoothing.cc"],
    hdrs = ["exponential_smoothing.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/control_component/controller_task_base/common:controllers_control_task_base_common",
    ],
)

apollo_cc_library(
    name = "interpolation_plus_1d",
    srcs = ["interpolation_plus_1d.cc"],
    hdrs = ["interpolation_plus_1d.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/control_component/controller_task_base/common:controllers_control_task_base_common",
        "@eigen",
    ],
)

apollo_cc_library(
    name = "leadlag_plus_controller",
    srcs = ["leadlag_plus_controller.cc"],
    hdrs = ["leadlag_plus_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/control_component/controller_task_base/common:controllers_control_task_base_common",
        "//modules/control/control_component/proto:leadlag_conf_cc_proto",
    ],
)

apollo_cc_library(
    name = "math_utils",
    srcs = ["math_utils.cc"],
    hdrs = ["math_utils.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/common/math",
        "@eigen",
    ]
)

apollo_cc_library(
    name = "pid_plus_controller",
    srcs = ["pid_plus_controller.cc"],
    hdrs = ["pid_plus_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/control_component/controller_task_base/common:controllers_control_task_base_common",
        "//modules/control/control_task_base_extend/proto:pid_plus_conf_proto",
    ],
)

apollo_cc_library(
    name = "trajectory_analyzer_extend",
    srcs = ["trajectory_analyzer_extend.cc"],
    hdrs = ["trajectory_analyzer_extend.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//modules/control/control_component/controller_task_base/common:controllers_control_task_base_common",
    ],
)


apollo_package()

cpplint()
