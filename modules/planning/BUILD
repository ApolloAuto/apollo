load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planning_component_lib",
    srcs = [
        "planning_component.cc",
    ],
    hdrs = [
        "planning_component.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":planning_lib",
        "//framework:cybertron",
        "//modules/common/util:message_util",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_library(
    name = "planning_lib",
    srcs = [
        "planning_base.cc",
    ],
    hdrs = [
        "planning_base.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:quaternion",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util:message_util",
        "//modules/localization/proto:localization_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:traffic_rule_config_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_library(
    name = "open_space_planning_lib",
    srcs = [
        "open_space_planning.cc",
    ],
    hdrs = [
        "open_space_planning.h",
    ],
    deps = [
        ":planning_lib",
    ],
)

cc_binary(
    name = "planning",
    srcs = [
        "main.cc",
        "planning.h",
    ],
    deps = [
        ":planning_lib",
        "//external:gflags",
    ],
)

filegroup(
    name = "planning_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

filegroup(
    name = "planning_conf",
    srcs = glob([
        "conf/**",
    ]),
)

cpplint()
