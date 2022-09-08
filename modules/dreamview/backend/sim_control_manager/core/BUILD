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
        "//modules/dreamview/backend/sim_control_manager/common:sim_control_gflags",
        "//modules/dreamview/backend/sim_control_manager/common:sim_control_util",
        "//modules/dreamview/backend/sim_control_manager/proto:sim_control_internal_cc_proto",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common/util:message_util",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/map:map_service",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/prediction_msgs:scenario_cc_proto",
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
        "//modules/dreamview/backend/sim_control_manager/common:sim_control_gflags",
        "//modules/dreamview/backend/sim_control_manager/common:sim_control_util",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_base",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/dreamview/backend/common:dreamview_gflags",
    ],
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
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_base",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_with_model_base",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/dreamview/backend/map:map_service",
    ],
)
