load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sim_control_base",
    srcs = [
        "sim_control_base.cc",
    ],
    hdrs = [
        "sim_control_base.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common/util:message_util",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/prediction_msgs:scenario_cc_proto",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/sim_control_manager/common:sim_control_gflags",
        "//modules/dreamview/backend/sim_control_manager/common:sim_control_util",
        "//modules/dreamview/backend/sim_control_manager/proto:sim_control_internal_cc_proto",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_library(
    name = "sim_control_with_model_base",
    srcs = [
        "sim_control_with_model_base.cc",
    ],
    hdrs = [
        "sim_control_with_model_base.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/sim_control_manager/common:sim_control_gflags",
        "//modules/dreamview/backend/sim_control_manager/common:sim_control_util",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_base",
        "//modules/dreamview/backend/sim_control_manager/dynamic_model/perfect_control:sim_perfect_control",
    ],
    alwayslink = True,
)

cc_library(
    name = "dynamic_model_factory",
    srcs = [
        "dynamic_model_factory.cc",
    ],
    hdrs = [
        "dynamic_model_factory.h",
    ],
    deps = [
        "//modules/common/status",
        "//modules/common/util",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_base",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_with_model_base",
        "//modules/dreamview/backend/sim_control_manager/proto:dynamic_model_conf_cc_proto",
    ],
)

cpplint()
