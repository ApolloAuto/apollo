load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_gflags",
    srcs = ["control_gflags.cc"],
    hdrs = ["control_gflags.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "hysteresis_filter",
    srcs = ["hysteresis_filter.cc"],
    hdrs = ["hysteresis_filter.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
)

cc_library(
    name = "interpolation_1d",
    srcs = ["interpolation_1d.cc"],
    hdrs = ["interpolation_1d.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "interpolation_2d",
    srcs = ["interpolation_2d.cc"],
    hdrs = ["interpolation_2d.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "leadlag_controller",
    srcs = ["leadlag_controller.cc"],
    hdrs = ["leadlag_controller.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        "//cyber",
        "//modules/control/proto:control_proto",
    ],
)

cc_library(
    name = "mrac_controller",
    srcs = ["mrac_controller.cc"],
    hdrs = ["mrac_controller.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        "//cyber",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/control/proto:control_proto",
        "@eigen",
    ],
)

cc_library(
    name = "pid_controller",
    srcs = ["pid_controller.cc"],
    hdrs = ["pid_controller.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        "//cyber",
        "//modules/control/proto:control_proto",
    ],
)

cc_library(
    name = "pid_BC_controller",
    srcs = ["pid_BC_controller.cc"],
    hdrs = ["pid_BC_controller.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/common/math",
        "//modules/control/proto:control_proto",
    ],
)

cc_library(
    name = "pid_IC_controller",
    srcs = ["pid_IC_controller.cc"],
    hdrs = ["pid_IC_controller.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/common/math",
        "//modules/control/proto:control_proto",
    ],
)

cc_library(
    name = "trajectory_analyzer",
    srcs = ["trajectory_analyzer.cc"],
    hdrs = ["trajectory_analyzer.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        ":control_gflags",
        "//cyber",
        "//modules/common/math:linear_interpolation",
        "//modules/common/math:search",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_library(
    name = "common",
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        ":control_gflags",
        ":hysteresis_filter",
        ":interpolation_1d",
        ":interpolation_2d",
        ":leadlag_controller",
        ":pid_controller",
        ":trajectory_analyzer",
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
    data = ["//modules/control/testdata:control_testdata"],
    deps = [
        ":interpolation_1d",
        "//cyber",
        "//modules/common/util",
        "//modules/control/proto:control_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "interpolation_2d_test",
    size = "small",
    srcs = ["interpolation_2d_test.cc"],
    data = ["//modules/control/testdata:control_testdata"],
    deps = [
        ":interpolation_2d",
        "//cyber",
        "//modules/common/util",
        "//modules/control/proto:control_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "leadlag_controller_test",
    size = "small",
    srcs = ["leadlag_controller_test.cc"],
    data = ["//modules/control/testdata:control_testdata"],
    deps = [
        ":leadlag_controller",
        "//modules/common/util",
        "//modules/control/proto:control_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "mrac_controller_test",
    size = "small",
    srcs = ["mrac_controller_test.cc"],
    data = ["//modules/control/testdata:control_testdata"],
    deps = [
        ":mrac_controller",
        "//modules/common/util",
        "//modules/control/proto:control_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pid_controller_test",
    size = "small",
    srcs = ["pid_controller_test.cc"],
    data = ["//modules/control/testdata:control_testdata"],
    deps = [
        ":pid_controller",
        "//modules/common/util",
        "//modules/control/proto:control_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pid_BC_controller_test",
    size = "small",
    srcs = ["pid_BC_controller_test.cc"],
    data = ["//modules/control/testdata:control_testdata"],
    deps = [
        ":pid_BC_controller",
        "//modules/common/util",
        "//modules/control/proto:control_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pid_IC_controller_test",
    size = "small",
    srcs = ["pid_IC_controller_test.cc"],
    data = ["//modules/control/testdata:control_testdata"],
    deps = [
        ":pid_IC_controller",
        "//modules/common/util",
        "//modules/control/proto:control_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "trajectory_analyzer_test",
    size = "small",
    srcs = ["trajectory_analyzer_test.cc"],
    deps = [
        ":trajectory_analyzer",
        "//modules/common/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
