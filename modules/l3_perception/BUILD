load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "l3_perception_util",
    srcs = ["l3_perception_util.cc"],
    hdrs = ["l3_perception_util.h"],
    deps = [
        ":l3_perception_gflags",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/common:log",
        "//modules/map/hdmap:hdmap_util"
    ],
)

cc_library(
    name = "l3_perception_lib",
    srcs = ["l3_perception.cc"],
    hdrs = ["l3_perception.h"],
    deps = [
        ":l3_perception_gflags",
        ":l3_perception_util",
        ":l3_perception_conversion",
        ":l3_perception_fusion",
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/proto:perception_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/proto:localization_proto",
    ],
)

cc_binary(
    name = "l3_perception",
    srcs = ["main.cc"],
    linkstatic = 0,
    deps = [
        ":l3_perception_lib",
        "//external:gflags",
        "//modules/common:log",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "l3_perception_gflags",
    srcs = [
        "l3_perception_gflags.cc",
    ],
    hdrs = [
        "l3_perception_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "l3_perception_conversion",
    srcs = [
        "conversion.cc",
    ],
    hdrs = [
        "conversion.h",
    ],
    deps = [
        ":l3_perception_gflags",
        ":l3_perception_util",
        "//modules/perception/proto:perception_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/l3_perception/proto:l3_perception_proto"
    ],
)

cc_library(
    name = "l3_perception_fusion",
    srcs = [
        "fusion.cc",
    ],
    hdrs = [
        "fusion.h",
    ],
    deps = [
        ":l3_perception_gflags",
        ":l3_perception_util",
        "//modules/common/math:polygon2d",
        "//modules/common/math:vec2d",
        "//modules/perception/proto:perception_proto",
    ],
)

cpplint()
