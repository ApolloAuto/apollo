load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dummy",
    srcs = [
        "dummy_algorithms.cc",
    ],
    hdrs = [
        "dummy_algorithms.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/common/sequence_type_fuser",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/lidar/interface",
        "@eigen",
        "@pcl",
    ],
)

cpplint()
