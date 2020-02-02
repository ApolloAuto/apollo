load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "poses_interpolation_lib",
    srcs = ["poses_interpolation.cc"],
    hdrs = ["poses_interpolation.h"],
    deps = [
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//third_party:boost",
        "@eigen",
    ],
)

cpplint()
