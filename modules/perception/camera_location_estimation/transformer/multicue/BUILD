load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obj_mapper",
    srcs = ["obj_mapper.cc"],
    hdrs = ["obj_mapper.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/camera/common",
        "@eigen",
    ],
)

cc_library(
    name = "multicue_transformer",
    srcs = ["multicue_transformer.cc"],
    hdrs = ["multicue_transformer.h"],
    deps = [
        ":obj_mapper",
        "//cyber",
        "//modules/perception/camera_location_estimation/interface:base_transformer",
        "//modules/perception/camera_location_estimation/transformer/multicue/proto:multicue_cc_proto",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
        "@eigen",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
