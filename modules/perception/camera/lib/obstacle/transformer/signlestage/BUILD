load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "signlestage_obstacle_transformer",
    srcs = ["signlestage_obstacle_transformer.cc"],
    hdrs = ["signlestage_obstacle_transformer.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/common:object_template_manager",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/obstacle/transformer/signlestage/proto:signlestage_cc_proto",
        "//modules/perception/common/i_lib/core",
    ],
)

cpplint()
