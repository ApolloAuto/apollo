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
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/dreamview/proto:data_collection_table_proto",
        "//third_party/json",
        "@gtest",
    ],
)

cpplint()
