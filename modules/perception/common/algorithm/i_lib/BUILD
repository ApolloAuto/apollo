load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "i_lib",
    deps = [
        "//modules/perception/common/algorithm/i_lib/algorithm:i_sort",
        "//modules/perception/common/algorithm/i_lib/core",
        "//modules/perception/common/algorithm/i_lib/da:i_ransac",
        "//modules/perception/common/algorithm/i_lib/geometry:i_plane",
        "//modules/perception/common/algorithm/i_lib/pc:i_ground",
    ],
)

apollo_package()
cpplint()
