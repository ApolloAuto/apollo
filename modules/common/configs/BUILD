load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "config_gflags",
    srcs = ["config_gflags.cc"],
    hdrs = ["config_gflags.h"],
    data = [
        "//modules/common/data:global_flagfile",
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "vehicle_config_helper",
    srcs = ["vehicle_config_helper.cc"],
    hdrs = ["vehicle_config_helper.h"],
    data = [
        "//modules/common/data:vehicle_config_data",
    ],
    deps = [
        ":config_gflags",
        "//cyber/common:file",
        "//cyber/common:macros",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/math:geometry",
        "//modules/common/proto:pnc_point_cc_proto",
    ],
)

cc_test(
    name = "vehicle_config_helper_test",
    size = "small",
    srcs = ["vehicle_config_helper_test.cc"],
    deps = [
        "vehicle_config_helper",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
