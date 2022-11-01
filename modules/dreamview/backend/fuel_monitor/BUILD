load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

cc_library(
    name = "fuel_monitor",
    hdrs = ["fuel_monitor.h"],
    deps = [
        "@com_github_nlohmann_json//:json",
    ],
)

cc_library(
    name = "data_collection_monitor",
    srcs = ["data_collection_monitor.cc"],
    hdrs = ["data_collection_monitor.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        ":fuel_monitor",
        ":fuel_monitor_gflags",
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/hmi:vehicle_manager",
        "//modules/dreamview/proto:data_collection_table_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_nlohmann_json//:json",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "data_collection_monitor_test",
    size = "small",
    srcs = ["data_collection_monitor_test.cc"],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":data_collection_monitor",
        "//modules/common/util:util_tool",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "preprocess_monitor",
    srcs = ["preprocess_monitor.cc"],
    hdrs = ["preprocess_monitor.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        ":fuel_monitor",
        ":fuel_monitor_gflags",
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/hmi:vehicle_manager",
        "//modules/dreamview/proto:preprocess_table_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "fuel_monitor_gflags",
    srcs = ["fuel_monitor_gflags.cc"],
    hdrs = ["fuel_monitor_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "fuel_monitor_manager",
    srcs = ["fuel_monitor_manager.cc"],
    hdrs = ["fuel_monitor_manager.h"],
    copts = DREAMVIEW_COPTS,
    deps = [
        ":fuel_monitor",
        "//cyber",
        "//modules/common/util:util_tool",
        "@boost",
    ],
)

cpplint()
