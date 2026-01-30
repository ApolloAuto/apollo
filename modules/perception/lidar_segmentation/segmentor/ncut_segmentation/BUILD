load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "ncut_segmentation",
    srcs = [
        "ncut_segmentation.cc",
        "ncut.cc",
    ],
    hdrs = [
        "ncut_segmentation.h",
        "ncut.h",
    ],
    copts = [
        "-fopenmp",
        #"-DDEBUG_NCUT",
    ],
    linkopts = [
        "-lgomp",
    ],
    deps = [
        "//cyber",
        "@eigen",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/lidar_segmentation/segmentor/ncut_segmentation/proto:ncut_config_cc_proto",
        "//modules/perception/lidar_segmentation/segmentor/ncut_segmentation/common:flood_fill",
        "//modules/perception/lidar_segmentation/segmentor/ncut_segmentation/common:lr_classifier",
        "//modules/perception/lidar_segmentation/segmentor/ncut_segmentation/proto:ncut_param_cc_proto",
        "//modules/perception/lidar_detection:apollo_perception_lidar_detection",
        "@local_config_pcl//:pcl",
        "@local_config_vtk//:vtk",
    ],
)

apollo_package(enable_source=False)
cpplint()
