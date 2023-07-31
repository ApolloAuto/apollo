load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interface",
    hdrs = [
        "base_data_association.h",
        "base_existence_fusion.h",
        "base_gatekeeper.h",
        "base_motion_fusion.h",
        "base_shape_fusion.h",
        "base_tracker.h",
        "base_type_fusion.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/multi_sensor_fusion/base:base_forward_declaration",
        "//modules/perception/multi_sensor_fusion/base:sensor",
    ],
)

cc_library(
    name = "base_fusion_system",
    hdrs = ["base_fusion_system.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base:frame",
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/multi_sensor_fusion/base",
    ],
)

apollo_package()
cpplint()
