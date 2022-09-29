load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "third_party_perception_radar_conversion",
    srcs = ["conversion_radar.cc"],
    hdrs = ["conversion_radar.h"],
    deps = [
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common_msgs/sensor_msgs:conti_radar_cc_proto",
        "//modules/common_msgs/sensor_msgs:delphi_esr_cc_proto",
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
    linkopts = ["-Wl,--allow-multiple-definition"],
    deps = [
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common_msgs/sensor_msgs:smartereye_cc_proto",
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
    linkopts = ["-Wl,--allow-multiple-definition"],
    deps = [
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common_msgs/sensor_msgs:mobileye_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/third_party_perception/common:third_party_perception_util",
    ],
)

cc_library(
    name = "third_party_perception_fusion",
    srcs = ["fusion.cc"],
    hdrs = ["fusion.h"],
    deps = [
        "//modules/common/math",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
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
