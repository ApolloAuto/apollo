load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_plugin", "apollo_cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libhdmap_roi_filter.so",
    description = ":plugins.xml",
    deps = [
        ":lib_hrf"
    ]
)

apollo_cc_library(
    name = "lib_hrf",
    srcs = [
        "bitmap2d.cc",
        "hdmap_roi_filter.cc",
    ],
    hdrs = [
        "bitmap2d.h",
        "hdmap_roi_filter.h",
        "polygon_mask.h",
        "polygon_scan_cvter.h"
    ],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//modules/perception/pointcloud_map_based_roi:apollo_perception_pointcloud_map_based_roi",
        "//cyber",
        "@boost",
        "@eigen",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/pointcloud_map_based_roi/roi_filter/hdmap_roi_filter/proto:hdmap_roi_filter_cc_proto",
        "//modules/perception/common/lib:apollo_perception_common_lib",
    ],
)

# apollo_cc_test(
#     name = "hdmap_roi_filter_test",
#     size = "small",
#     srcs = ["hdmap_roi_filter_test.cc"],
#     deps = [
#         ":hdmap_roi_filter_lib",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

apollo_package()
cpplint()
