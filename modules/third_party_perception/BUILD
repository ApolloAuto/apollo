load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_third_party_perception",
    srcs = [
        "common/third_party_perception_gflags.cc",
        "common/third_party_perception_util.cc",
        "integration_tests/third_party_perception_test_base.cc",
        "third_party_perception_base.cc",
        "third_party_perception_mobileye.cc",
        "third_party_perception_smartereye.cc",
        "tools/conversion_mobileye.cc",
        "tools/conversion_radar.cc",
        "tools/conversion_smartereye.cc",
        "tools/filter.cc",
        "tools/fusion.cc",
    ],
    hdrs = [
        "common/third_party_perception_gflags.h",
        "common/third_party_perception_util.h",
        "integration_tests/third_party_perception_test_base.h",
        "third_party_perception_base.h",
        "third_party_perception_mobileye.h",
        "third_party_perception_smartereye.h",
        "tools/conversion_mobileye.h",
        "tools/conversion_radar.h",
        "tools/conversion_smartereye.h",
        "tools/filter.h",
        "tools/fusion.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/sensor_msgs:conti_radar_cc_proto",
        "//modules/common_msgs/sensor_msgs:delphi_esr_cc_proto",
        "//modules/common_msgs/sensor_msgs:mobileye_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/sensor_msgs:smartereye_cc_proto",
        "//modules/map:apollo_map",
        "//modules/third_party_perception/proto:radar_obstacle_cc_proto",
        "//modules/third_party_perception/proto:third_party_perception_component_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest",
    ],
)

apollo_component(
    name = "libthird_party_perception_component.so",
    srcs = ["third_party_perception_component.cc"],
    hdrs = ["third_party_perception_component.h"],
    copts = ['-DMODULE_NAME=\\"third_party_perception\\"'],
    deps = [
        ":apollo_third_party_perception",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_package()

cpplint()
