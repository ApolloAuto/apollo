load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "third_party_perception_lib",
    srcs = ["third_party_perception.cc"],
    hdrs = ["third_party_perception.h"],
    deps = [
        ":third_party_perception_conversion",
        ":third_party_perception_filter",
        ":third_party_perception_fusion",
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/third_party_perception/common:third_party_perception_gflags",
        "//modules/third_party_perception/common:third_party_perception_util",
    ],
)

cc_binary(
    name = "third_party_perception",
    srcs = ["main.cc"],
    deps = [
        ":third_party_perception_lib",
        "//external:gflags",
        "//modules/common:log",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "third_party_perception_conversion",
    srcs = [
        "conversion.cc",
    ],
    hdrs = [
        "conversion.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/configs:config_gflags",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/third_party_perception/common:third_party_perception_gflags",
        "//modules/third_party_perception/common:third_party_perception_util",
        "//modules/third_party_perception/proto:third_party_perception_proto",
    ],
)

cc_library(
    name = "third_party_perception_fusion",
    srcs = [
        "fusion.cc",
    ],
    hdrs = [
        "fusion.h",
    ],
    deps = [
        "//modules/common/math:geometry",
        "//modules/perception/proto:perception_proto",
        "//modules/third_party_perception/common:third_party_perception_gflags",
        "//modules/third_party_perception/common:third_party_perception_util",
    ],
)

cc_library(
    name = "third_party_perception_filter",
    srcs = [
        "filter.cc",
    ],
    hdrs = [
        "filter.h",
    ],
    deps = [
        "//modules/third_party_perception/common:third_party_perception_gflags",
        "//modules/third_party_perception/common:third_party_perception_util",
        "//modules/third_party_perception/proto:third_party_perception_proto",
    ],
)

cpplint()
