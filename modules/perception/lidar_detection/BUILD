load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "point_pillars_common",
    hdrs = ["detector/point_pillars_detection/common.h"],
)

gpu_library(
    name = "anchor_mask_cuda",
    srcs = ["detector/point_pillars_detection/anchor_mask_cuda.cu"],
    hdrs = ["detector/point_pillars_detection/anchor_mask_cuda.h"],
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
    srcs = ["detector/point_pillars_detection/nms_cuda.cu"],
    hdrs = ["detector/point_pillars_detection/nms_cuda.h"],
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
    srcs = ["detector/point_pillars_detection/pfe_cuda.cu"],
    hdrs = ["detector/point_pillars_detection/pfe_cuda.h"],
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
    srcs = ["detector/point_pillars_detection/postprocess_cuda.cu"],
    hdrs = ["detector/point_pillars_detection/postprocess_cuda.h"],
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
    srcs = ["detector/point_pillars_detection/preprocess_points_cuda.cu"],
    hdrs = ["detector/point_pillars_detection/preprocess_points_cuda.h"],
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
    srcs = ["detector/point_pillars_detection/scatter_cuda.cu"],
    hdrs = ["detector/point_pillars_detection/scatter_cuda.h"],
    deps = [
        ":point_pillars_common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

gpu_library(
    name = "feature_generator_cuda",
    srcs = ["detector/cnn_segmentation/feature_generator.cu"],
    hdrs = ["detector/cnn_segmentation/feature_generator.h"],
    deps = [
        ":cnn_segmentation_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/lidar_detection/detector/cnn_segmentation/proto:model_param_cc_proto",
        "@com_google_googletest//:gtest",
        "@eigen",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

apollo_cc_library(
    name = "cnn_segmentation_util",
    hdrs = ["detector/cnn_segmentation/util.h"],
)

apollo_cc_library(
    name = "apollo_perception_lidar_detection",
    srcs = [
        "detector/center_point_detection/center_point_detection.cc",
        "detector/cnn_segmentation/cnn_segmentation.cc",
        "detector/cnn_segmentation/feature_generator.cc",
        "detector/cnn_segmentation/spp_engine/spp_cluster_list.cc",
        "detector/cnn_segmentation/spp_engine/spp_engine.cc",
        "detector/cnn_segmentation/spp_engine/spp_label_image.cc",
        "detector/cnn_segmentation/spp_engine/spp_pool_types.cc",
        "detector/cnn_segmentation/spp_engine/spp_seg_cc_2d.cc",
        "detector/cnn_segmentation/spp_engine/spp_struct.cc",
        "detector/mask_pillars_detection/mask_pillars_detection.cc",
        "detector/point_pillars_detection/point_pillars.cc",
        "detector/point_pillars_detection/point_pillars_detection.cc",
        "detector/point_pillars_detection/preprocess_points.cc",
    ],
    hdrs = [
        "detector/center_point_detection/center_point_detection.h",
        "detector/center_point_detection/params.h",
        "detector/cnn_segmentation/cnn_segmentation.h",
        "detector/cnn_segmentation/disjoint_set.h",
        "detector/cnn_segmentation/feature_generator.h",
        "detector/cnn_segmentation/spp_engine/spp_cluster.h",
        "detector/cnn_segmentation/spp_engine/spp_cluster_list.h",
        "detector/cnn_segmentation/spp_engine/spp_engine.h",
        "detector/cnn_segmentation/spp_engine/spp_label_image.h",
        "detector/cnn_segmentation/spp_engine/spp_pool_types.h",
        "detector/cnn_segmentation/spp_engine/spp_seg_cc_2d.h",
        "detector/cnn_segmentation/spp_engine/spp_struct.h",
        "detector/mask_pillars_detection/mask_pillars_detection.h",
        "detector/point_pillars_detection/params.h",
        "detector/point_pillars_detection/point_pillars.h",
        "detector/point_pillars_detection/point_pillars_detection.h",
        "detector/point_pillars_detection/preprocess_points.h",
        "interface/base_lidar_detector.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        ":anchor_mask_cuda",
        ":feature_generator_cuda",
        ":nms_cuda",
        ":pfe_cuda",
        ":point_pillars_postprocess_cuda",
        ":preprocess_points_cuda",
        ":scatter_cuda",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "//modules/perception/common/interface:base_down_sample",
        "//modules/perception/lidar_detection/detector/center_point_detection/proto:center_point_model_param_cc_proto",
        "//modules/perception/lidar_detection/detector/cnn_segmentation/proto:model_param_cc_proto",
        "//modules/perception/lidar_detection/detector/mask_pillars_detection/proto:model_param_cc_proto",
        "//modules/perception/lidar_detection/detector/point_pillars_detection/proto:model_param_cc_proto",
        "//modules/perception/lidar_detection/down_sample_bank/pcl_down_sample/proto:pcl_down_sample_proto",
        "//modules/perception/lidar_detection/proto:lidar_detection_component_config_cc_proto",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest",
        "@eigen",
        "@local_config_pcl//:pcl",
        "@libtorch_gpu",
    ] + select({
        "@platforms//cpu:x86_64": ["@centerpoint_infer_op-x86_64//:centerpoint_infer_op"],
        "@platforms//cpu:aarch64": ["@centerpoint_infer_op-aarch64//:centerpoint_infer_op"],
    }) + select({
        "@platforms//cpu:x86_64": ["@paddleinference-x86_64//:paddleinference_lib"],
        "@platforms//cpu:aarch64": ["@paddleinference-aarch64//:paddleinference_lib"],
    }) + if_cuda([
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:migraphx",
    ]),
)

apollo_component(
    name = "liblidar_detection_component.so",
    srcs = ["lidar_detection_component.cc"],
    hdrs = ["lidar_detection_component.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    linkopts = ["-lleveldb"],
    deps = [
        ":apollo_perception_lidar_detection",
        "//cyber",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/lidar_detection/proto:lidar_detection_component_config_cc_proto",
    ],
)

apollo_cc_test(
    name = "cnn_segmentation_test",
    size = "large",
    srcs = ["detector/cnn_segmentation/cnn_segmentation_test.cc"],
    deps = [
        ":apollo_perception_lidar_detection",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
