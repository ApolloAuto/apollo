load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "copts_if_teleop", "if_teleop")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

apollo_cc_test(
    name = "websocket_handler_test",
    size = "small",
    srcs = ["handlers/websocket_handler_test.cc"],
    deps = [
        ":dreamview_common",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "map_service_test",
    size = "small",
    srcs = ["map_service/map_service_test.cc"],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":dreamview_common",
        "//modules/common/configs:config_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "data_collection_monitor_test",
    size = "small",
    srcs = ["fuel_monitor/data_collection_monitor_test.cc"],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":dreamview_common",
        "//modules/common/util:util_tool",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "teleop_test",
    size = "small",
    srcs = if_teleop(
        ["teleop/teleop_test.cc"],
    ),
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":dreamview_common",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_binary(
    name = "vehicle_manager_main",
    srcs = ["vehicle_manager/vehicle_manager_main.cc"],
    copts = DREAMVIEW_COPTS,
    deps = [
        ":dreamview_common",
        "//cyber",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_test(
    name = "vehicle_manager_test",
    size = "small",
    srcs = ["vehicle_manager/vehicle_manager_test.cc"],
    data = glob(["vehicle_manager/testdata/**"]),
    deps = [
        ":dreamview_common",
        "//cyber",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_library(
    name = "dreamview_common",
    copts = DREAMVIEW_COPTS,
    srcs = [
        "dreamview_gflags.cc",
        "util/hmi_util.cc",
        "handlers/image_handler.cc",
        "handlers/proto_handler.cc",
        "handlers/websocket_handler.cc",
        "map_service/map_service.cc",
        "sim_control_manager/common/interpolation_2d.cc",
        "sim_control_manager/common/sim_control_gflags.cc",
        "sim_control_manager/common/sim_control_util.cc",
        "sim_control_manager/core/dynamic_model_factory.cc",
        "sim_control_manager/core/sim_control_base.cc",
        "sim_control_manager/core/sim_control_with_model_base.cc",
        "sim_control_manager/dynamic_model/perfect_control/sim_perfect_control.cc",
        "sim_control_manager/sim_control_manager.cc",
        "vehicle_manager/vehicle_manager.cc",
        "fuel_monitor/data_collection_monitor.cc",
        "fuel_monitor/fuel_monitor_gflags.cc",
        "fuel_monitor/fuel_monitor_manager.cc",
        "fuel_monitor/preprocess_monitor.cc",
        "plugins/plugin_manager.cc",
    ]+ if_teleop(["teleop.cc"]),
    hdrs = [
        "dreamview_gflags.h",
        "util/hmi_util.h",
        "handlers/image_handler.h",
        "handlers/proto_handler.h",
        "handlers/websocket_handler.h",
        "map_service/map_service.h",
        "sim_control_manager/common/interpolation_2d.h",
        "sim_control_manager/common/sim_control_gflags.h",
        "sim_control_manager/common/sim_control_util.h",
        "sim_control_manager/core/dynamic_model_factory.h",
        "sim_control_manager/core/sim_control_base.h",
        "sim_control_manager/core/sim_control_with_model_base.h",
        "sim_control_manager/dynamic_model/perfect_control/sim_perfect_control.h",
        "sim_control_manager/sim_control_manager.h",
        "vehicle_manager/vehicle_manager.h",
        "fuel_monitor/data_collection_monitor.h",
        "fuel_monitor/fuel_monitor.h",
        "fuel_monitor/fuel_monitor_gflags.h",
        "fuel_monitor/fuel_monitor_manager.h",
        "fuel_monitor/preprocess_monitor.h",
        "plugins/plugin_manager.h",
    ]+ if_teleop(["teleop.h"]),
    deps = [
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
        "//cyber/common:cyber_common",
        "//modules/common_msgs/dreamview_msgs:hmi_config_cc_proto",
        "//modules/common_msgs/dreamview_msgs:hmi_mode_cc_proto",
        "//modules/common_msgs/dreamview_msgs:simulation_world_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/map:apollo_map",
        "//modules/dreamview/backend/common/sim_control_manager/proto:dynamic_model_conf_cc_proto",
        "//modules/dreamview/backend/common/sim_control_manager/proto:sim_control_internal_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/dreamview/proto:data_collection_table_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/dreamview/proto:preprocess_table_cc_proto",
        "//modules/dreamview/proto:dv_plugin_msg_cc_proto",
        "//modules/dreamview/proto:plugin_config_cc_proto",
        "@com_github_nlohmann_json//:json",
        "@opencv//:imgcodecs",
        "@civetweb//:civetweb++",
    ] + if_teleop([
        "//modules/teleop/modem/proto:modem_info_cc_proto",
        "//modules/teleop/daemon/proto:daemon_cmd_cc_proto",
        "//modules/teleop/daemon/proto:daemon_rpt_cc_proto",
    ]),
)

apollo_package()

cpplint()
