load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "location_refiner_obstacle_postprocessor",
    srcs = ["location_refiner_obstacle_postprocessor.cc"],
    hdrs = ["location_refiner_obstacle_postprocessor.h"],
    deps = [
        ":obj_postprocessor",
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/obstacle/postprocessor/location_refiner/proto:location_refiner_cc_proto",
    ],
)

cc_library(
    name = "obj_postprocessor",
    srcs = ["obj_postprocessor.cc"],
    hdrs = ["obj_postprocessor.h"],
    deps = [
        "//modules/perception/camera/common",
    ],
)

cpplint()
