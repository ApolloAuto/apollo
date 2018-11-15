load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "feature_extractor",
    srcs = ["feature_extractor.cc"],
    hdrs = ["feature_extractor.h"],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/common/math:geometry",
        "//modules/map/hdmap:hdmap_util",
        "//modules/prediction/proto:scenario_feature_proto",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/adc_trajectory:adc_trajectory_container",
        "//modules/prediction/container/pose:pose_container",
        "@gtest//:main",
    ],
)

cc_test(
    name = "feature_extractor_test",
    size = "small",
    srcs = ["feature_extractor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container:container_manager",
        ":feature_extractor",
        "@gtest//:main",
    ],
)

cpplint()
