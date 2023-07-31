load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "i_plane",
    hdrs = [
        "i_line.h",
        "i_plane.h",
        "i_util.h",
    ],
    alwayslink = True,
    deps = [
        "//modules/perception/common/algorithm/i_lib/algorithm:i_sort",
        "//modules/perception/common/algorithm/i_lib/core",
        "//modules/perception/common/algorithm/i_lib/da:i_ransac",
        "@eigen",
    ],
)

apollo_package()
cpplint()
