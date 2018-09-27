load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dst_existance_fusion",
    srcs = [
        "dst_existance_fusion.cc",
    ],
    hdrs = [
        "dst_existance_fusion.h",
    ],
    deps = [
       "//modules/perception/fusion/base:base",
       "//modules/perception/fusion/common:camera_util",
       "//modules/perception/fusion/common:dst_evidence",
       "//modules/perception/fusion/lib/interface:interface",
       "//modules/perception/lib/io:protobuf_util",
       "//modules/perception/lib/registerer:registerer",
       "//modules/perception/proto:dst_existance_fusion_config_proto",
    ],
)

cc_test(
    name = "dst_existance_fusion_test",
    size = "small",
    srcs = [
        "dst_existance_fusion_test.cc",
    ],
    deps = [
        ":dst_existance_fusion",
        "@gtest//:main",
    ],
)

cpplint()
