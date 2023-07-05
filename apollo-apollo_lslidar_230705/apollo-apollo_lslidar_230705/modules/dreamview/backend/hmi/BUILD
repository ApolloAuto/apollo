load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

install(
    name = "install",
    library_dest = "dreamview/lib",
    runtime_dest = "dreamview/bin",
    targets = [
        ":libhmi_worker.so",
        ":vehicle_manager_main",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "vehicle_manager",
    srcs = ["vehicle_manager.cc"],
    hdrs = ["vehicle_manager.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/dreamview/proto:hmi_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
    ],
)

cc_binary(
    name = "vehicle_manager_main",
    srcs = ["vehicle_manager_main.cc"],
    copts = DREAMVIEW_COPTS,
    deps = [
        ":vehicle_manager",
        "//cyber",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "vehicle_manager_test",
    size = "small",
    srcs = ["vehicle_manager_test.cc"],
    data = glob(["testdata/**"]),
    deps = [
        ":vehicle_manager",
        "//cyber",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "hmi",
    srcs = ["hmi.cc"],
    hdrs = ["hmi.h"],
    copts = DREAMVIEW_COPTS,
    alwayslink = True,
    deps = [
        ":hmi_worker",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/monitor_log",
        "//modules/common/util:util_tool",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/fuel_monitor:fuel_monitor_manager",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/backend/point_cloud:point_cloud_updater",
        "//modules/dreamview/proto:preprocess_table_cc_proto",
    ],
)

cc_binary(
    name = "libhmi_worker.so",
    srcs = [
        "hmi_worker.cc",
        "hmi_worker.h",
    ],
    linkshared = True,
    linkstatic = True,
    copts = DREAMVIEW_COPTS,
    deps = [
        ":vehicle_manager",
        "//cyber",
        "//cyber/proto:dag_conf_cc_proto",
        "//modules/common_msgs/audio_msgs:audio_event_cc_proto",
        "//modules/common_msgs/basic_msgs:drive_event_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:pad_msg_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/monitor_msgs:system_status_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/common/kv_db",
        "//modules/common/util:util_tool",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/fuel_monitor:data_collection_monitor",
        "//modules/dreamview/backend/fuel_monitor:fuel_monitor_gflags",
        "//modules/dreamview/backend/fuel_monitor:fuel_monitor_manager",
        "//modules/dreamview/backend/fuel_monitor:preprocess_monitor",
        "//modules/dreamview/proto:hmi_config_cc_proto",
        "//modules/common_msgs/dreamview_msgs:hmi_status_cc_proto",
        "//modules/dreamview/proto:scenario_cc_proto",
        "//modules/dreamview/proto:record_cc_proto",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "@boost",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "hmi_worker",
    srcs = ["libhmi_worker.so"],
    hdrs = ["hmi_worker.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        "//cyber",
        "//cyber/proto:dag_conf_cc_proto",
        "//modules/common_msgs/audio_msgs:audio_event_cc_proto",
        "//modules/common_msgs/basic_msgs:drive_event_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:pad_msg_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/kv_db",
        "//modules/common/util:util_tool",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/proto:hmi_config_cc_proto",
        "//modules/common_msgs/dreamview_msgs:hmi_status_cc_proto",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/dreamview/proto:record_cc_proto",
        "@boost",
        "@com_google_absl//:absl",
    ],
)

cc_test(
    name = "hmi_worker_test",
    size = "small",
    srcs = ["hmi_worker_test.cc"],
    deps = [
        ":hmi_worker",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
