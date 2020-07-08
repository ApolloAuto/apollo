load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "third_party_perception_radar_conversion",
    srcs = ["conversion_radar.cc"],
    hdrs = ["conversion_radar.h"],
    deps = [
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/proto:drive_state_cc_proto",
        "//modules/common/proto:vehicle_signal_cc_proto",
        "//modules/drivers/canbus/proto:can_card_parameter_cc_proto",
        "//modules/drivers/proto:conti_radar_cc_proto",
        "//modules/drivers/proto:delphi_esr_cc_proto",
        "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_cc_proto",
        "//modules/third_party_perception/common:third_party_perception_util",
        "//modules/third_party_perception/proto:radar_obstacle_cc_proto",
    ],
)

cc_library(
    name = "third_party_perception_smartereye_conversion",
    srcs = ["conversion_smartereye.cc"],
    hdrs = [
        "conversion_base.h",
        "conversion_smartereye.h",
    ],
    deps = [
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/proto:drive_state_cc_proto",
        "//modules/common/proto:vehicle_signal_cc_proto",
        "//modules/drivers/proto:smartereye_cc_proto",
        "//modules/third_party_perception/common:third_party_perception_util",
    ],
)

cc_library(
    name = "third_party_perception_mobileye_conversion",
    srcs = ["conversion_mobileye.cc"],
    hdrs = [
        "conversion_base.h",
        "conversion_mobileye.h",
    ],
    deps = [
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/proto:drive_state_cc_proto",
        "//modules/common/proto:vehicle_signal_cc_proto",
        "//modules/drivers/proto:mobileye_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/third_party_perception/common:third_party_perception_util",
    ],
)

cc_library(
    name = "third_party_perception_fusion",
    srcs = ["fusion.cc"],
    hdrs = ["fusion.h"],
    deps = [
        "//modules/common/math:geometry",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/third_party_perception/proto:third_party_perception_component_cc_proto",
    ],
)

cc_library(
    name = "third_party_perception_filter",
    srcs = ["filter.cc"],
    hdrs = ["filter.h"],
    deps = [
        "//modules/third_party_perception/common:third_party_perception_util",
        "//modules/third_party_perception/proto:radar_obstacle_cc_proto",
    ],
)

cpplint()
