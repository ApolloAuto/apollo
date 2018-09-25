load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cloud_mask",
    srcs = [
        "cloud_mask.cc",
    ],
    hdrs = [
        "cloud_mask.h",
    ],
    deps = [
        "//modules/perception/base:point_cloud",
    ],
)

# cc_library(
#     name = "base_type",
#     hdrs = [
#         "hdmap_struct.h",
#         "lane_struct.h",
#         "landmark_struct.h",
#         "sensor_meta.h",
#         "vehicle_struct.h",
#     ],
# )
#
# cc_test(
#     name = "syncedmem_test",
#     size = "small",
#     srcs = [
#         "syncedmem_test.cc",
#     ],
#     deps = [
#         ":syncedmem",
#         "//modules/perception/base/test:test_helper",
#         "@cuda",
#         "@gtest//:main",
#     ],
# )

cpplint()
