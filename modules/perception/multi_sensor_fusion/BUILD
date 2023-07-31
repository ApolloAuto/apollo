load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "multi_sensor_fusion_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
    ])
)

cc_binary(
    name = "libmulti_sensor_fusion.so",
    linkshared = True,
    linkstatic = True,
    deps = [":multi_sensor_fusion_component"],
)

cc_library(
    name = "multi_sensor_fusion_component",
    srcs = ["multi_sensor_fusion_component.cc"],
    hdrs = ["multi_sensor_fusion_component.h"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common/base",
        "//modules/perception/multi_sensor_fusion/fusion/fusion_system/probabilistic_fusion",
        "//modules/perception/multi_sensor_fusion/interface",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/hdmap:hdmap_input",
        "//modules/perception/common/onboard/common_flags",
        "//modules/perception/common/onboard/inner_component_messages",
        "//modules/perception/common/onboard/msg_serializer",
        "//modules/perception/multi_sensor_fusion/proto:fusion_component_config_cc_proto",
        "@eigen",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
