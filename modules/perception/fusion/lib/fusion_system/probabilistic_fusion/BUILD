load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "probabilistic_fusion",
    srcs = ["probabilistic_fusion.cc"],
    hdrs = ["probabilistic_fusion.h"],
    deps = [
        "//cyber",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/perception/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/graph",
        "//modules/perception/fusion/base",
        "//modules/perception/fusion/common:camera_util",
        "//modules/perception/fusion/common:dst_evidence",
        "//modules/perception/fusion/lib/data_association/hm_data_association:chi_squared_cdf",
        "//modules/perception/fusion/lib/data_association/hm_data_association:hm_tracks_objects_match",
        "//modules/perception/fusion/lib/data_association/hm_data_association:track_object_distance",
        "//modules/perception/fusion/lib/data_fusion/existance_fusion/dst_existance_fusion",
        "//modules/perception/fusion/lib/data_fusion/tracker/pbf_tracker",
        "//modules/perception/fusion/lib/data_fusion/type_fusion/dst_type_fusion",
        "//modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/thread",
        "//modules/perception/lib/utils",
        "//modules/perception/proto:pbf_tracker_config_proto",
        "//modules/perception/proto:perception_config_schema_proto",
        "//modules/perception/proto:probabilistic_fusion_config_proto",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "probabilistic_fusion_test",
    size = "small",
    srcs = ["probabilistic_fusion_test.cc"],
    deps = [
        ":probabilistic_fusion",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
