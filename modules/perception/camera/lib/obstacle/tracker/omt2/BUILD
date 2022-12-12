load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "omt_bev_tracker",
    srcs = ["omt_bev_tracker.cc"],
    hdrs = ["omt_bev_tracker.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/obstacle/tracker/omt:target",
        "//modules/perception/common/geometry:common",
        "//modules/perception/pipeline/proto/stage:omt_cc_proto",
    ],
)

cpplint()
