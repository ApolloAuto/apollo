load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "singlestage_transformer",
    srcs = ["singlestage_transformer.cc"],
    hdrs = ["singlestage_transformer.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera_location_estimation/interface:base_transformer",
        "//modules/perception/camera_location_estimation/transformer/singlestage/proto:singlestage_cc_proto",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
        "@eigen",
    ],
)

apollo_package()
cpplint()
