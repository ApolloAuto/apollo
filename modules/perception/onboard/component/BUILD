load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libperception_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":perception_component_inner"],
)

cc_library(
    name = "perception_component_inner",
    srcs = [
        "fusion_component.cc",
        "lidar_output_component.cc",
        "recognition_component.cc",
        "camera_perception_viz_message.cc",
        "fusion_camera_detection_component.cc",
        "trafficlights_perception_component.cc",
        "segmentation_component.cc",
    ],
    hdrs = [
        "fusion_component.h",
        "lidar_inner_component_messages.h",
        "camera_perception_viz_message.h",
        "fusion_camera_detection_component.h",
        "trafficlights_perception_component.h",
        "lidar_output_component.h",
        "recognition_component.h",
        "segmentation_component.h",
    ],
    copts = [
        '-DMODULE_NAME=\\"perception\\"',
    ],
    deps = [
        "//cybertron",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:geometry_proto",
        "//modules/common/proto:header_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/common/math:math",
        "//modules/common/time:time",
        "//modules/perception/base",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/camera/common:common",
        "//modules/perception/lib/utils",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/singleton:singleton",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lib/io:protobuf_util",
        "//modules/perception/common:perception_gflags",
        "//modules/drivers/proto:sensor_proto",
        "//modules/map/proto:map_proto",

        "//modules/perception/fusion/lib/dummy:dummy_algorithms",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lidar/app",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
        "//modules/perception/lidar/lib/scene_manager/ground_service",
        "//modules/perception/lidar/lib/scene_manager/roi_service",
        "//modules/perception/lidar/lib/segmentation/cnnseg:cnn_segmentation",

        "//modules/perception/lidar/lib/classifier/fused_classifier",
        "//modules/perception/lidar/lib/classifier/fused_classifier:ccrf_type_fusion",
        "//modules/perception/fusion/app:obstacle_multi_sensor_fusion",
        "//modules/perception/fusion/base",
        "//modules/perception/camera/app:obstacle_camera_perception",
        "//modules/perception/camera/app:traffic_light_camera_perception",
        "//modules/perception/camera/lib/traffic_light/preprocessor:tl_preprocessor",
        "//modules/perception/camera/lib/traffic_light/preprocessor:multi_camera_projection",
        "//modules/perception/camera/lib/traffic_light/preprocessor:pose",

        "//modules/perception/onboard/proto:lidar_component_config_proto",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_engine",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_track_object_matcher",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_tracker",


        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/msg_serializer",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/perception/onboard/proto:fusion_camera_detection_proto",
        "//modules/perception/onboard/proto:trafficlights_perception_component_proto",
        "//modules/transform:tf2_buffer_lib",
        "//modules/transform/proto:transform_proto",
        "//modules/perception/proto:perception_proto",
        "@eigen",
    ],
)

cpplint()
