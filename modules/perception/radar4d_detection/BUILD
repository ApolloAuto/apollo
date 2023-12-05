load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "radar4d_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "point_pillars_common",
    hdrs = ["lib/detector/point_pillars_detection/common.h"],
)

gpu_library(
    name = "anchor_mask_cuda",
    srcs = ["lib/detector/point_pillars_detection/anchor_mask_cuda.cu"],
    hdrs = ["lib/detector/point_pillars_detection/anchor_mask_cuda.h"],
    deps = [
        ":point_pillars_common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

gpu_library(
    name = "nms_cuda",
    srcs = ["lib/detector/point_pillars_detection/nms_cuda.cu"],
    hdrs = ["lib/detector/point_pillars_detection/nms_cuda.h"],
    deps = [
        ":point_pillars_common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

gpu_library(
    name = "pfe_cuda",
    srcs = ["lib/detector/point_pillars_detection/pfe_cuda.cu"],
    hdrs = ["lib/detector/point_pillars_detection/pfe_cuda.h"],
    deps = [
        ":point_pillars_common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

gpu_library(
    name = "point_pillars_postprocess_cuda",
    srcs = ["lib/detector/point_pillars_detection/postprocess_cuda.cu"],
    hdrs = ["lib/detector/point_pillars_detection/postprocess_cuda.h"],
    deps = [
        ":nms_cuda",
        ":point_pillars_common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

gpu_library(
    name = "preprocess_points_cuda",
    srcs = ["lib/detector/point_pillars_detection/preprocess_points_cuda.cu"],
    hdrs = ["lib/detector/point_pillars_detection/preprocess_points_cuda.h"],
    deps = [
        ":point_pillars_common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

gpu_library(
    name = "scatter_cuda",
    srcs = ["lib/detector/point_pillars_detection/scatter_cuda.cu"],
    hdrs = ["lib/detector/point_pillars_detection/scatter_cuda.h"],
    deps = [
        ":point_pillars_common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_radar4d_detection",
    srcs = [
        "app/radar_obstacle_perception.cc",
        "lib/classifier/ccrf_type_fusion.cc",
        "lib/classifier/fused_classifier.cc",
        "lib/classifier/util.cc",
        "lib/detector/point_pillars_detection/point_pillars.cc",
        "lib/detector/point_pillars_detection/point_pillars_detection.cc",
        "lib/detector/point_pillars_detection/preprocess_points.cc",
        "lib/object_builder/object_builder.cc",
        "lib/preprocessor/radar_preprocessor.cc",
        "lib/roi_filter/hdmap_radar_roi_filter.cc",
        "lib/tracker/association/distance_collection.cc",
        "lib/tracker/association/gnn_bipartite_graph_matcher.cc",
        "lib/tracker/association/multi_hm_bipartite_graph_matcher.cc",
        "lib/tracker/common/mrf_track_data.cc",
        "lib/tracker/common/track_data.cc",
        "lib/tracker/common/track_pool_types.cc",
        "lib/tracker/common/tracked_object.cc",
        "lib/tracker/measurement/measurement_collection.cc",
        "lib/tracker/multi_radar_fusion/mrf_engine.cc",
        "lib/tracker/multi_radar_fusion/mrf_motion_filter.cc",
        "lib/tracker/multi_radar_fusion/mrf_motion_measurement.cc",
        "lib/tracker/multi_radar_fusion/mrf_motion_refiner.cc",
        "lib/tracker/multi_radar_fusion/mrf_shape_filter.cc",
        "lib/tracker/multi_radar_fusion/mrf_track_object_distance.cc",
        "lib/tracker/multi_radar_fusion/mrf_track_object_matcher.cc",
        "lib/tracker/multi_radar_fusion/mrf_tracker.cc",
        "lib/tracker/semantic_map/evaluator_manager.cc",
    ],
    hdrs = [
        "app/radar_obstacle_perception.h",
        "interface/base_bipartite_graph_matcher.h",
        "interface/base_classifier.h",
        "interface/base_detector.h",
        "interface/base_multi_target_tracker.h",
        "interface/base_obstacle_perception.h",
        "interface/base_preprocessor.h",
        "interface/base_roi_filter.h",
        "lib/classifier/ccrf_type_fusion.h",
        "lib/classifier/fused_classifier.h",
        "lib/classifier/type_fusion_interface.h",
        "lib/classifier/util.h",
        "lib/detector/point_pillars_detection/params.h",
        "lib/detector/point_pillars_detection/point_pillars.h",
        "lib/detector/point_pillars_detection/point_pillars_detection.h",
        "lib/detector/point_pillars_detection/preprocess_points.h",
        "lib/object_builder/object_builder.h",
        "lib/preprocessor/radar_preprocessor.h",
        "lib/roi_filter/hdmap_radar_roi_filter.h",
        "lib/tracker/association/distance_collection.h",
        "lib/tracker/association/gnn_bipartite_graph_matcher.h",
        "lib/tracker/association/multi_hm_bipartite_graph_matcher.h",
        "lib/tracker/common/mrf_track_data.h",
        "lib/tracker/common/track_data.h",
        "lib/tracker/common/track_pool_types.h",
        "lib/tracker/common/tracked_object.h",
        "lib/tracker/measurement/measurement_collection.h",
        "lib/tracker/multi_radar_fusion/mrf_base_filter.h",
        "lib/tracker/multi_radar_fusion/mrf_engine.h",
        "lib/tracker/multi_radar_fusion/mrf_motion_filter.h",
        "lib/tracker/multi_radar_fusion/mrf_motion_measurement.h",
        "lib/tracker/multi_radar_fusion/mrf_motion_refiner.h",
        "lib/tracker/multi_radar_fusion/mrf_shape_filter.h",
        "lib/tracker/multi_radar_fusion/mrf_track_object_distance.h",
        "lib/tracker/multi_radar_fusion/mrf_track_object_matcher.h",
        "lib/tracker/multi_radar_fusion/mrf_tracker.h",
        "lib/tracker/semantic_map/evaluator_manager.h",
    ],
    deps = [
        ":anchor_mask_cuda",
        ":nms_cuda",
        ":pfe_cuda",
        ":point_pillars_postprocess_cuda",
        ":preprocess_points_cuda",
        ":scatter_cuda",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/sensor_msgs:oculii_radar_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/hdmap:apollo_perception_common_hdmap",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "//modules/perception/common/radar:apollo_perception_common_radar",
        "//modules/perception/radar4d_detection/lib/classifier/proto:ccrf_type_fusion_config_cc_proto",
        "//modules/perception/radar4d_detection/lib/classifier/proto:fused_classifier_config_cc_proto",
        "//modules/perception/radar4d_detection/lib/detector/point_pillars_detection/proto:model_param_cc_proto",
        "//modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/proto:mrf_config_cc_proto",
        "//modules/perception/radar4d_detection/proto:preprocessor_config_cc_proto",
        "//modules/perception/radar4d_detection/proto:radar4d_component_config_cc_proto",
        "//modules/perception/radar4d_detection/proto:radar_obstacle_perception_config_cc_proto",
        "//modules/prediction:apollo_prediction",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_pcl//:pcl",
        "@local_config_tensorrt//:tensorrt",
        "@libtorch_gpu",
    ],
)

apollo_component(
    name = "libradar4d_detection.so",
    srcs = ["radar4d_detection_component.cc"],
    hdrs = ["radar4d_detection_component.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    linkopts = ["-lleveldb"],
    deps = [
        ":apollo_perception_radar4d_detection",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/hdmap:apollo_perception_common_hdmap",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/radar4d_detection/proto:radar4d_component_config_cc_proto",
    ],
)

apollo_package()

cpplint()
