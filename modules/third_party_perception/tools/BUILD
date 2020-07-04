load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "third_party_perception_radar_conversion",
    srcs = ["conversion_radar.cc"],
    hdrs = ["conversion_radar.h"],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/third_party_perception/common:third_party_perception_util",
        "//modules/third_party_perception/proto:third_party_perception_proto",
    ],
)

cc_library(
    name = "third_party_perception_smartereye_conversion",
    srcs = ["conversion_smartereye.cc"],
    hdrs = [
        "conversion_smartereye.h",
        "conversion_base.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/third_party_perception/common:third_party_perception_util",
        "//modules/third_party_perception/proto:third_party_perception_proto",
    ],
)

cc_library(
    name = "third_party_perception_mobileye_conversion",
    srcs = ["conversion_mobileye.cc"],
    hdrs = [
        "conversion_mobileye.h",
        "conversion_base.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/third_party_perception/common:third_party_perception_util",
        "//modules/third_party_perception/proto:third_party_perception_proto",
    ],
)

cc_library(
    name = "third_party_perception_fusion",
    srcs = ["fusion.cc"],
    hdrs = ["fusion.h"],
    deps = [
        "//modules/common/math:geometry",
        "//modules/perception/proto:perception_proto",
    ],
)

cc_library(
    name = "third_party_perception_filter",
    srcs = ["filter.cc"],
    hdrs = ["filter.h"],
    deps = [
        "//modules/third_party_perception/common:third_party_perception_util",
        "//modules/third_party_perception/proto:third_party_perception_proto",
    ],
)

cpplint()
