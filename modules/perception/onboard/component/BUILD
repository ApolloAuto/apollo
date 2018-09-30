load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libperception_component.so",
    deps = [":perception_component_inner"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "perception_component_inner",
    srcs = [
        "fusion_component.cc",
        "lidar_output_component.cc",
        "lidar_common_flags.cc",
        "segmentation_component.cc",
        "recognition_component.cc",
    ],
    hdrs = [
        "fusion_component.h",
        "lidar_output_component.h",
        "lidar_common_flags.h",
        "segmentation_component.h",
        "recognition_component.h",
        "lidar_inner_component_messages.h",
    ],
    deps = [
        "@eigen",
        "//framework:cybertron",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:geometry_proto",
        "//modules/common/proto:header_proto",
        "//modules/map/proto:map_proto",
        "//modules/perception/base",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/lib/utils",
        "//modules/perception/lib/registerer",
        "//modules/perception/map/hdmap:hdmap_input",
        "//modules/perception/lidar/app",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/scene_manager/ground_service:ground_service",
        "//modules/perception/lidar/lib/scene_manager/roi_service:roi_service",
        "//modules/perception/lidar/lib/segmentation/cnnseg:cnn_segmentation",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector",
        "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
        "//modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter",
        "//modules/perception/lidar/lib/tracker/hm_tracker:hm_multi_target_tracker",
        "//modules/perception/lidar/lib/tracker/hm_tracker:kalman_filter",
        "//modules/perception/lidar/lib/tracker/hm_tracker:multi_hm_bipartite_graph_matcher",
        "//modules/perception/lidar/lib/classifier/fused_classifier",
        "//modules/perception/lidar/lib/classifier/fused_classifier:ccrf_type_fusion",
        "//modules/perception/fusion/app:obstacle_multi_sensor_fusion",
        "//modules/perception/fusion/base",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/fusion/lib/dummy:dummy_algorithms",
        "//modules/perception/fusion/lib/fusion_system/probabilistic_fusion",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/onboard/msg_serializer",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/transform:tf2_buffer_lib",
    ],
    copts = ['-DMODULE_NAME=\\"perception\\"']
)

cpplint()
