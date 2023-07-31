load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_library(
    name = "base_postprocessor",
    hdrs = [
        "base_postprocessor.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common/lib/interface:base_calibration_service",
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
    ]
)

apollo_package()
cpplint()
