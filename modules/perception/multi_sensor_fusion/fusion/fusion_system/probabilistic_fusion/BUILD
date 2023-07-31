load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "probabilistic_fusion",
    srcs = ["probabilistic_fusion.cc"],
    hdrs = ["probabilistic_fusion.h"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common/util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common:util",
        "//modules/perception/common/algorithm/graph",
        "//modules/perception/common/base",
        "//modules/perception/common/lib/thread",
        "//modules/perception/multi_sensor_fusion/base",
        "//modules/perception/multi_sensor_fusion/common:camera_util",
        "//modules/perception/multi_sensor_fusion/common:dst_evidence",
        "//modules/perception/multi_sensor_fusion/fusion/data_association/hm_data_association:chi_squared_cdf",
        "//modules/perception/multi_sensor_fusion/fusion/data_association/hm_data_association:hm_tracks_objects_match",
        "//modules/perception/multi_sensor_fusion/fusion/data_association/hm_data_association:track_object_distance",
        "//modules/perception/multi_sensor_fusion/fusion/data_fusion/existence_fusion/dst_existence_fusion",
        "//modules/perception/multi_sensor_fusion/fusion/data_fusion/tracker/pbf_tracker",
        "//modules/perception/multi_sensor_fusion/fusion/data_fusion/type_fusion/dst_type_fusion",
        "//modules/perception/multi_sensor_fusion/fusion/gatekeeper/pbf_gatekeeper",
        "//modules/perception/multi_sensor_fusion/interface:base_fusion_system",
        "//modules/perception/multi_sensor_fusion/interface",
        "//modules/perception/multi_sensor_fusion/proto:probabilistic_fusion_config_cc_proto",
        "@eigen",
    ],
    alwayslink = True,
)

# ignore temporarily TODO:// need fix logic

# cc_test(
#     name = "probabilistic_fusion_test",
#     size = "small",
#     srcs = ["probabilistic_fusion_test.cc"],
#     copts = ["-fno-access-control"],
#     deps = [
#         ":probabilistic_fusion",
#         "@com_google_googletest//:gtest_main",
#     ],
#     linkstatic = True,
# )

apollo_package()
cpplint()
