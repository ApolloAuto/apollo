load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_localization",
    srcs = [
        "camera_localization.cc",
    ],
    hdrs = [
        "camera_localization.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/localization:localization_base",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:camera_parameter_proto",
        "//modules/localization/proto:camera_proto",
        "//modules/localization/proto:localization_config_proto",
        "//modules/localization/proto:localization_proto",
        "//third_party/ros:ros_common",
    ],
)

cpplint()
