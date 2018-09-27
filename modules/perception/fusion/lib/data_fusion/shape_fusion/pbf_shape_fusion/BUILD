load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pbf_shape_fusion",
    srcs = [
        "pbf_shape_fusion.cc",
    ],
    hdrs = [
        "pbf_shape_fusion.h",
    ],
    deps = [
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/fusion/base:track",
        "//modules/perception/fusion/base:scene",
        "//modules/perception/fusion/lib/interface:interface",
    ],
)

cpplint()
