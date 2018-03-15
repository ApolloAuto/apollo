load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_manager",
    srcs = ["vehicle_manager.cc"],
    hdrs = ["vehicle_manager.h"],
    deps = [
        "//modules/common/util",
        "//modules/common/util:string_util",
        "//modules/dreamview/proto:hmi_config_proto",
    ],
)

cc_binary(
    name = "vehicle_manager_main",
    srcs = ["vehicle_manager_main.cc"],
    deps = [
        ":vehicle_manager",
    ],
)

cc_library(
    name = "hmi",
    srcs = ["hmi.cc"],
    hdrs = ["hmi.h"],
    deps = [
        ":hmi_worker",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/monitor_log",
        "//modules/common/util:json_util",
        "//modules/common/util:map_util",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/backend/map:map_service",
    ],
)

cc_library(
    name = "hmi_worker",
    srcs = ["hmi_worker.cc"],
    hdrs = ["hmi_worker.h"],
    deps = [
        ":vehicle_manager",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/kv_db",
        "//modules/common/util:map_util",
        "//modules/data/util:info_collector",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/map:map_service",
        "//modules/dreamview/proto:hmi_config_proto",
        "//modules/dreamview/proto:hmi_status_proto",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "hmi_worker_test",
    size = "small",
    srcs = ["hmi_worker_test.cc"],
    deps = [
        ":hmi_worker",
        "@gtest//:main",
    ],
)

cpplint()
