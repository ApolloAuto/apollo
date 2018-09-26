load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = [
        "util.cc",
    ],
    hdrs = [
        "util.h",
    ],
    deps = [
        "//modules/perception/base:object",
        "//framework:cybertron",
        "@eigen",
    ],
)

cc_library(
    name = "type_fusion_interface",
    hdrs = [
        "type_fusion_interface.h",
    ],
    deps = [
        "//modules/perception/base:object",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/lidar/common:common",
    ],
)

cpplint()
