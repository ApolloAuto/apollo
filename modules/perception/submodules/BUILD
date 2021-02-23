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
        ":camera_detection_submodule",
        ":lane_detection_submodule",
        ":motion_submodule",
        ":traffic_light_submodule",
    ],
    alwayslink = True,
)

cc_library(
  name = "camera_detection_submodule",
  srcs = ["camera_detection_submodule.cc"],
  hdrs = ["camera_detection_submodule.h"],
  deps = [
        ":camera_perception_viz_message",
        ":common_flags",
        ":inner_submodule_messages",
        ":transform_wrapper",
        "//cyber",
        "//modules/common/math:math_utils",
        "//modules/common/util:eigen_defs",
        "//modules/common/util:string_util",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/perception/base",
        "//modules/perception/camera/app:cipv_camera",
        "//modules/perception/camera/app:obstacle_camera_perception",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface:base_camera_perception",
        "//modules/perception/camera/tools/offline:visualizer",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/proto:camera_detection_submodule_cc_proto",
        "//modules/perception/proto:motion_service_cc_proto",
        "//modules/perception/proto:perception_camera_cc_proto",
        "//modules/perception/proto:perception_obstacle_cc_proto",
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
    name = "traffic_light_submodule",
    srcs = ["traffic_light_submodule.cc"],
    hdrs = ["traffic_light_submodule.h"],
    deps = [
        ":common_flags",
        ":transform_wrapper",
        "//cyber",
        "//modules/common/math:math_utils",
        "//modules/common/util:util",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/map/proto:map_geometry_cc_proto",
        "//modules/map/proto:map_signal_cc_proto",
        "//modules/perception/camera/app:traffic_light_camera_perception",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/traffic_light/preprocessor:tl_preprocessor",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/perception/proto:traffic_light_detection_cc_proto",
        "//modules/perception/proto:traffic_light_submodule_cc_proto",
        "//modules/transform:buffer",
        "//modules/transform/proto:transform_cc_proto",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:v2x_traffic_light_cc_proto",
        "@boost",
        "@com_google_absl//absl/strings",
        "@opencv//:highgui",
        "@opencv//:imgproc",
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
        ":lidar_segmentation_submodule",
        ":lidar_tracking_submodule",
        ":fusion_submodule",
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
    name = "lidar_segmentation_submodule",
    srcs = ["lidar_segmentation_submodule.cc"],
    hdrs = ["lidar_segmentation_submodule.h"],
    deps = [
        ":common_flags",
        ":lidar_inner_submodule_messages",
        ":transform_wrapper",
        "//cyber/time:clock",
        "//modules/common/util:string_util",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_segmentation",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
        "//modules/perception/lidar/lib/scene_manager/ground_service",
        "//modules/perception/lidar/lib/scene_manager/roi_service",
        "//modules/perception/lidar/lib/segmentation/cnnseg:cnn_segmentation",
        "//modules/perception/lidar/lib/segmentation/ncut:ncut_segmentation",
        "//modules/perception/proto:lidar_submodule_config_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "lidar_tracking_submodule",
    srcs = [
        "lidar_tracking_submodule.cc",
    ],
    hdrs = [
        "lidar_tracking_submodule.h",
    ],
    deps = [
        ":lidar_inner_submodule_messages",
        "//cyber/time:clock",
        "//modules/common/util:perf_util",
        "//modules/perception/base",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/app:lidar_obstacle_tracking",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_engine",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_track_object_matcher",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_tracker",
        "//modules/perception/proto:lidar_submodule_config_cc_proto",
    ],
)

cc_library(
    name = "fusion_submodule",
    srcs = ["fusion_submodule.cc"],
    hdrs = ["fusion_submodule.h"],
    deps = [
        ":common_flags",
        ":inner_submodule_messages",
        ":msg_serializer",
        "//cyber/time:clock",
        "//modules/common/util:perf_util",
        "//modules/perception/base",
        "//modules/perception/fusion/app:obstacle_multi_sensor_fusion",
        "//modules/perception/fusion/lib/dummy:dummy_algorithms",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/lib/classifier/fused_classifier",
        "//modules/perception/lidar/lib/classifier/fused_classifier:ccrf_type_fusion",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/proto:fusion_submodule_config_cc_proto",
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

cc_library(
    name = "msg_serializer",
    srcs = ["msg_serializer.cc"],
    hdrs = ["msg_serializer.h"],
    deps = [
        ":common_flags",
        "//cyber/time:clock",
        "//modules/perception/base",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/prediction/proto:feature_cc_proto",
        "@eigen",
    ],
)

cpplint()
