load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "config_gflags",
    srcs = [
        "config_gflags.cc",
    ],
    hdrs = [
        "config_gflags.h",
    ],
    data = [
        "//modules/common/data:global_flagfile",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "vehicle_config_helper",
    srcs = [
        "vehicle_config_helper.cc",
    ],
    hdrs = [
        "vehicle_config_helper.h",
    ],
    data = [
        "//modules/common/data:vehicle_config_data",
    ],
    deps = [
        ":config_gflags",
        "//modules/common",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/util",
        "@glog//:glog",
    ],
)

cc_library(
    name = "cpu_bind_helper",
    srcs = [
        "cpu_bind_helper.cc",
    ],
    hdrs = [
        "cpu_bind_helper.h",
    ],
    deps = [
        ":config_gflags",
        "//modules/common",
        "@yaml_cpp//:yaml",
    ],
)

cc_test(
    name = "vehicle_config_helper_test",
    size = "small",
    srcs = [
        "vehicle_config_helper_test.cc",
    ],
    deps = [
        "vehicle_config_helper",
        "@gtest//:main",
    ],
)

cc_test(
    name = "cpu_bind_helper_test",
    size = "small",
    srcs = [
        "cpu_bind_helper_test.cc",
    ],
    deps = [
        "cpu_bind_helper",
        "@gtest//:main",
    ],
)

cpplint()
