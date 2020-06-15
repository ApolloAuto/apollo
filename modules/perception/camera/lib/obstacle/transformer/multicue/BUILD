load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obj_mapper",
    srcs = ["obj_mapper.cc"],
    hdrs = ["obj_mapper.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/common/proto:object_template_meta_schema_cc_proto",
    ],
)

cc_library(
    name = "multicue_obstacle_transformer",
    srcs = ["multicue_obstacle_transformer.cc"],
    hdrs = ["multicue_obstacle_transformer.h"],
    deps = [
        ":obj_mapper",
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/obstacle/transformer/multicue/proto:multicue_cc_proto",
    ],
)

cpplint()
