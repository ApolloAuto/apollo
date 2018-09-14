load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_lib",
    srcs = [
    ],
    deps = [
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
        "//framework:cybertron",
#        "//external:gflags",
    ],
)

cc_library(
    name = "control_component_lib",
    srcs = [
        "control_component.cc",
    ],
    hdrs = [
        "control_component.h",
    ],
    deps = [
        ":control_lib",
    ],
    copts = ['-DMODULE_NAME=\\"control\\"']
)

cc_binary(
    name = "libcontrol_component.so",
    deps = [":control_component_lib"],
    linkopts = ["-shared"],
)



cpplint()
