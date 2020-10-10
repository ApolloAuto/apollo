load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interface",
    hdrs = [
        "base_data_association.h",
        "base_existence_fusion.h",
        "base_fusion_system.h",
        "base_gatekeeper.h",
        "base_motion_fusion.h",
        "base_shape_fusion.h",
        "base_tracker.h",
        "base_type_fusion.h",
    ],
    deps = [
        "//modules/perception/fusion/base:base_forward_declaration",
        "//modules/perception/fusion/base:sensor",
        "//modules/perception/lib/registerer",
    ],
)

cpplint()
