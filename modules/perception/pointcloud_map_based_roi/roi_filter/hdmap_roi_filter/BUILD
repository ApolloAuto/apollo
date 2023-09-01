load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libhdmap_roi_filter.so",
    description = ":plugins.xml",
    deps = [
        ":lib_hdmap_roi_filter"
    ]
)

apollo_cc_library(
    name = "lib_hdmap_roi_filter",
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
