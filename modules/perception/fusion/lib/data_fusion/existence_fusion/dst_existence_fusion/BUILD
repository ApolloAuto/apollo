load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dst_existence_fusion",
    srcs = ["dst_existence_fusion.cc"],
    hdrs = ["dst_existence_fusion.h"],
    deps = [
        "//modules/common/util",
        "//modules/perception/fusion/base",
        "//modules/perception/fusion/common:camera_util",
        "//modules/perception/fusion/common:dst_evidence",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/registerer",
        "//modules/perception/proto:dst_existence_fusion_config_cc_proto",
    ],
)

cc_test(
    name = "dst_existence_fusion_test",
    size = "small",
    srcs = ["dst_existence_fusion_test.cc"],
    deps = [
        ":dst_existence_fusion",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
