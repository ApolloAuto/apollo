load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "i_lib",
    deps = [
        "//modules/perception/common/i_lib/algorithm:i_sort",
        "//modules/perception/common/i_lib/core",
        "//modules/perception/common/i_lib/da:i_ransac",
        "//modules/perception/common/i_lib/geometry:i_plane",
        "//modules/perception/common/i_lib/pc:i_ground",
    ],
)

cpplint()
