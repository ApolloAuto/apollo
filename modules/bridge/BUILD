load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bridge_control_lib",
    srcs = [
    ],
    copts = ['-DMODULE_NAME=\\"bridge\\"'],
    deps = [
        "//cyber",
        "//external:gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/control/common",
        "//modules/control/controller",
        "//modules/control/proto:control_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_library(
    name = "bridge_control_component_lib",
    srcs = [
        "bridge_control_component.cc",
    ],
    hdrs = [
        "bridge_control_component.h",
    ],
    copts = ['-DMODULE_NAME=\\"bridge\\"'],
    deps = [
        ":bridge_control_lib",
    ],
)

cc_binary(
    name = "libbridge_control_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":bridge_control_component_lib"],
)

#cc_test(
#    name = "bridge_control_component_test",
#    size = "small",
#    srcs = [
#        "bridge_control_component_test.cc",
#    ],
#    data = ["//modules/control:control_testdata"],
#    deps = [
#        ":bridge_control_component_lib",
#        "@gtest//:main",
#    ],
#)

#filegroup(
#    name = "control_testdata",
#    srcs = glob([
#        "testdata/**",
#    ]),
#)

cpplint()
