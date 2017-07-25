package(default_visibility = ["//visibility:public"])

load("//tools:cpplint.bzl", "cpplint")

cc_library(
    name = "perception_proxy",
    srcs = [
        "perception_proxy.cc",
    ],
    hdrs = [
        "perception_proxy.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "routing_proxy",
    srcs = [
        "routing_proxy.cc",
    ],
    hdrs = [
        "routing_proxy.h",
    ],
    data = ["//modules/map:map_data"],
    deps = [
        "//external:gflags",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
    ],
)

cc_library(
    name = "vehicle_state_proxy",
    srcs = [
        "vehicle_state_proxy.cc",
    ],
    hdrs = [
        "vehicle_state_proxy.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:path_point_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/common:planning_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/localization/proto:localization_proto",
    ],
)

cpplint()
