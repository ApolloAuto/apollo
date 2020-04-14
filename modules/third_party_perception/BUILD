load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "third_party_perception_lib",
    srcs = ["third_party_perception_base.cc"],
    hdrs = ["third_party_perception_base.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/third_party_perception/tools:third_party_perception_filter",
        "//modules/third_party_perception/tools:third_party_perception_fusion",
        "//modules/third_party_perception/tools:third_party_perception_radar_conversion",
        "//modules/third_party_perception/tools:third_party_perception_smartereye_conversion",
        "//modules/third_party_perception/tools:third_party_perception_mobileye_conversion",               
    ],
)

cc_library(
    name = "third_party_perception_mobileye_lib",
    srcs = ["third_party_perception_mobileye.cc"],
    hdrs = ["third_party_perception_mobileye.h"],
    deps = [
        ":third_party_perception_lib",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/third_party_perception/tools:third_party_perception_filter",
        "//modules/third_party_perception/tools:third_party_perception_fusion",
        "//modules/third_party_perception/tools:third_party_perception_radar_conversion",
        "//modules/third_party_perception/tools:third_party_perception_smartereye_conversion",
        "//modules/third_party_perception/tools:third_party_perception_mobileye_conversion",
    ],
)

cc_library(
    name = "third_party_perception_smartereye_lib",
    srcs = ["third_party_perception_smartereye.cc"],
    hdrs = ["third_party_perception_smartereye.h"],
    deps = [
        ":third_party_perception_lib",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/third_party_perception/tools:third_party_perception_filter",
        "//modules/third_party_perception/tools:third_party_perception_fusion",
        "//modules/third_party_perception/tools:third_party_perception_radar_conversion",
        "//modules/third_party_perception/tools:third_party_perception_smartereye_conversion",
        "//modules/third_party_perception/tools:third_party_perception_mobileye_conversion",
    ],
)

cc_library(
    name = "third_party_perception_component_lib",
    srcs = ["third_party_perception_component.cc"],
    hdrs = ["third_party_perception_component.h"],
    copts = ['-DMODULE_NAME=\\"third_party_perception\\"'],
    deps = [
        ":third_party_perception_lib",
        ":third_party_perception_mobileye_lib",
        ":third_party_perception_smartereye_lib",
    ],
)

cc_binary(
    name = "libthird_party_perception_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":third_party_perception_component_lib"],
)

cpplint()
