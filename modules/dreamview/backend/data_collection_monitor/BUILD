load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_collection_monitor",
    srcs = [
        "data_collection_monitor.cc",
    ],
    hdrs = [
        "data_collection_monitor.h",
    ],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//cyber",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/proto:data_collection_table_proto",
        "//third_party/json",
        "@gtest",
    ],
)

cc_test(
    name = "data_collection_monitor_test",
    size = "small",
    srcs = [
        "data_collection_monitor_test.cc",
    ],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":data_collection_monitor",
        "//modules/common/util:json_util",
        "@gtest//:main",
    ],
)

cpplint()
