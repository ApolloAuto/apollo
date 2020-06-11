load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "i_ground",
    srcs = ["i_ground.cc"],
    hdrs = ["i_ground.h"],
    copts = ["-msse4.1"],
    deps = [
        ":i_struct_s",
        ":i_util",
        "//modules/perception/common/i_lib/algorithm:i_sort",
        "//modules/perception/common/i_lib/core",
        "//modules/perception/common/i_lib/da:i_ransac",
        "//modules/perception/common/i_lib/geometry:i_plane",
    ],
)

cc_library(
    name = "i_struct_s",
    hdrs = ["i_struct_s.h"],
)

cc_library(
    name = "i_util",
    hdrs = ["i_util.h"],
)

cpplint()
