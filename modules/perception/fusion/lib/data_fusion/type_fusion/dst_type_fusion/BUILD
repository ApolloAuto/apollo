load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dst_type_fusion",
    srcs = [
        "dst_type_fusion.cc",
    ],
    hdrs = [
        "dst_type_fusion.h",
    ],
    deps = [
        "//modules/common/util:file_util",
        "//modules/perception/proto:dst_type_fusion_config_proto",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/fusion/common:dst_evidence",
        "//modules/perception/fusion/lib/interface:interface",
        "//modules/perception/fusion/base:scene",
        "//modules/perception/fusion/base:track",
        "//modules/perception/fusion/base:base_init_options",
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/fusion/common:camera_util",
    ],
)

cpplint()
