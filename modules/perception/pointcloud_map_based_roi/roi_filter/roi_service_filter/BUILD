load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_plugin", "apollo_cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libroi_service_filter.so",
    description = ":plugins.xml",
    deps = [":lib_rsf"],
)

apollo_cc_library(
    name = "lib_rsf",
    srcs = [
        "roi_service_filter.cc",
    ],
    hdrs = [
        "roi_service_filter.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//modules/perception/pointcloud_map_based_roi:apollo_perception_pointcloud_map_based_roi",
        "//cyber",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "@eigen",
    ],
)

# apollo_cc_test(
#     name = "roi_service_filter_test",
#     size = "small",
#     srcs = ["roi_service_filter_test.cc"],
#     deps = [
#         ":roi_service_filter_lib",
#         "//modules/perception/common:perception_gflags",
#         "//modules/perception/common/io:io_util",
#         "//modules/perception/lidar/common:pcl_util",
#         "//modules/perception/lidar/lib/roi_filter/hdmap_roi_filter",
#         "//modules/perception/map/hdmap:apollo_perception_common_hdmap",
#         "//modules/perception/lidar/lib/scene_manager/ground_service:ground_service",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

apollo_package()
cpplint()
