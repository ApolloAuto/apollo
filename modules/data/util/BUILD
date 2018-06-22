load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "info_collector",
    srcs = ["info_collector.cc"],
    hdrs = ["info_collector.h"],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common/kv_db",
        "//modules/common/util",
        "//modules/data/proto:static_info_proto",
        "@yaml_cpp//:yaml",
    ],
)

cpplint()
