load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_base",
    srcs = [
        "localization_base.cc",
    ],
    hdrs = [
        "localization_base.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/configs:config_gflags",
        "//modules/common/status",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_proto",
        "@ros//:ros_common",
    ],
)

filegroup(
    name = "localization_testdata",
    srcs = glob(["testdata/*"]),
)

cpplint()
