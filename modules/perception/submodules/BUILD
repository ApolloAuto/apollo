load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

install(
    name = "install",
    targets = [
        ":libperception_submodule_camera.so",
        ":libperception_submodule_lidar.so",
    ],
    deps = [
        "//cyber:install",
        "//modules/perception/production:install",
    ],
)

cc_binary(
    name = "libperception_submodule_camera.so",
    linkshared = True,
    linkstatic = False,
    deps = [":perception_submodule_inner_camera"],
)

cc_library(
    name = "perception_submodule_inner_camera",
    copts = PERCEPTION_COPTS,
    deps = [
        ":motion_submodule",
        ":lane_detection_submodule",
    ],
    alwayslink = True,
)

cc_library(
    name = "lane_detection_submodule",
    srcs = ["lane_detection_submodule.cc"],
    hdrs = ["lane_detection_submodule.h"],
    deps = [
        ":camera_perception_viz_message",
        ":common_flags",
        ":inner_submodule_messages",
        ":transform_wrapper",
        "//cyber",
        "//cyber/time:clock",
        "//modules/common/math",
        "//modules/common/proto:error_code_cc_proto",
        "//modules/common/proto:geometry_cc_proto",
        "//modules/common/proto:header_cc_proto",
        "//modules/common/util",
        "//modules/common/util:eigen_defs",
        "//modules/common/util:time_util",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/map/proto:map_cc_proto",
        "//modules/perception/base",
        "//modules/perception/camera/app:lane_camera_perception",
        "//modules/perception/camera/common",
        "//modules/perception/camera/tools/offline:transform_server",
        "//modules/perception/camera/tools/offline:visualizer",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/proto:lane_detection_submodule_cc_proto",
        "//modules/perception/proto:perception_lane_cc_proto",
        "//modules/transform/proto:transform_cc_proto",
        "@eigen",
        "@opencv//:highgui",
    ],
    alwayslink = True,
)

cc_library(
    name = "motion_submodule",
    srcs = ["motion_submodule.cc"],
    hdrs = ["motion_submodule.h"],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/localization/proto:localization_status_cc_proto",
        "//modules/localization/proto:pose_cc_proto",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface:base_init_options",
        "//modules/perception/camera/lib/motion",
        "//modules/perception/lib/registerer",
        "//modules/perception/proto:motion_service_cc_proto",
    ],
)

cc_library(
    name = "camera_perception_viz_message",
    srcs = ["camera_perception_viz_message.cc"],
    hdrs = ["camera_perception_viz_message.h"],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/proto:error_code_cc_proto",
        "//modules/perception/base",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libperception_submodule_lidar.so",
    linkshared = True,
    linkstatic = False,
    deps = [":perception_submodule_inner_lidar"],
)

cc_library(
    name = "perception_submodule_inner_lidar",
    copts = PERCEPTION_COPTS,
    deps = [
        ":lidar_detection_submodule",
    ],
    alwayslink = True,
)

cc_library(
    name = "lidar_detection_submodule",
    srcs = ["lidar_detection_submodule.cc"],
    hdrs = ["lidar_detection_submodule.h"],
    deps = [
        ":common_flags",
        ":lidar_inner_submodule_messages",
        ":transform_wrapper",
        "//cyber/time:clock",
        "//modules/common/util:string_util",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_detection",
        "//modules/perception/lidar/common",
        "//modules/perception/proto:lidar_submodule_config_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "common_flags",
    srcs = ["common_flags.cc"],
    hdrs = ["common_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "inner_submodule_messages",
    hdrs = ["inner_submodule_messages.h"],
)

cc_library(
    name = "lidar_inner_submodule_messages",
    hdrs = [
        "lidar_inner_submodule_messages.h",
    ],
    deps = [
        ":inner_submodule_messages",
        "//cyber",
        "//modules/perception/lidar/common",
    ],
)

cc_library(
    name = "transform_wrapper",
    srcs = ["transform_wrapper.cc"],
    hdrs = ["transform_wrapper.h"],
    deps = [
        "//modules/common/util:string_util",
        "//modules/perception/common/sensor_manager",
        "//modules/transform:buffer",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cpplint()
