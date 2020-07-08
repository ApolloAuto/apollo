load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_common",
    srcs = [
        "localization_gflags.cc",
    ],
    hdrs = [
        "localization_gflags.h",
    ],
    deps = [
        "//cyber",
        "//modules/dreamview/backend/hmi:vehicle_manager",
        "//modules/localization/proto:localization_cc_proto",
    ],
)

cpplint()
