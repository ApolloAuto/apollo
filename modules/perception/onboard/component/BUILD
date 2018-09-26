load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lidar_output_component",
    srcs = [
        "lidar_output_component.cc",
    ],
    hdrs = [
        "lidar_output_component.h",
    ],
    deps = [
        "@eigen",
        "//framework:cybertron",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/base",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/perception/onboard/msg_serializer",
    ],
)

cc_library(
    name = "lidar_common_flags",
    srcs = [
        "lidar_common_flags.cc",
    ],
    hdrs = [
        "lidar_common_flags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "lidar_inner_component_messages",
    hdrs = [
        "lidar_inner_component_messages.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "segmentation_component",
    srcs = [
        "segmentation_component.cc",
    ],
    hdrs = [
        "segmentation_component.h",
    ],
    deps = [
        ":lidar_common_flags",
        ":lidar_inner_component_messages",
        "@eigen",
        "//framework:cybertron",
        "//modules/drivers/proto:sensor_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/base",
        "//modules/perception/lib/utils",
        "//modules/perception/lidar/app",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/transform:tf2_buffer_lib",
    ],
)

cc_library(
    name = "recognition_component",
    srcs = [
        "recognition_component.cc",
    ],
    hdrs = [
        "recognition_component.h",
    ],
    deps = [
        ":lidar_common_flags",
        ":lidar_inner_component_messages",
        "@eigen",
        "//framework:cybertron",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:geometry_proto",
        "//modules/common/proto:header_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/base",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/utils",
        "//modules/perception/lidar/app",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/perception/proto:perception_proto",
    ],
)

cpplint()
