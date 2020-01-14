load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dummy_algorithms",
    srcs = [
        "dummy_classifier.cc",
        "dummy_ground_detector.cc",
        "dummy_multi_target_tracker.cc",
        "dummy_object_filter.cc",
        "dummy_roi_filter.cc",
        "dummy_segmentation.cc",
    ],
    hdrs = [
        "dummy_classifier.h",
        "dummy_ground_detector.h",
        "dummy_multi_target_tracker.h",
        "dummy_object_filter.h",
        "dummy_roi_filter.h",
        "dummy_segmentation.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/common/point_cloud_processing",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "@eigen",
    ],
)

cc_test(
    name = "dummy_algorithms_test",
    size = "small",
    srcs = ["dummy_algorithms_test.cc"],
    deps = [
        ":dummy_algorithms",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
