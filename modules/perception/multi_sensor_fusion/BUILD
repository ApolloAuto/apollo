load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "multi_sensor_fusion_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
    ]),
)

apollo_cc_library(
    name = "apollo_perception_multi_sensor_fusion",
    srcs = [
        "base/scene.cc",
        "base/sensor.cc",
        "base/sensor_data_manager.cc",
        "base/sensor_frame.cc",
        "base/sensor_object.cc",
        "base/track.cc",
        "base/track_pool_types.cc",
        "common/camera_util.cc",
        "common/dst_evidence.cc",
        "common/information_filter.cc",
        "common/kalman_filter.cc",
        "fusion/data_association/hm_data_association/hm_tracks_objects_match.cc",
        "fusion/data_association/hm_data_association/probabilities.cc",
        "fusion/data_association/hm_data_association/track_object_distance.cc",
        "fusion/data_association/hm_data_association/track_object_similarity.cc",
        "fusion/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.cc",
        "fusion/data_fusion/motion_fusion/kalman_motion_fusion/kalman_motion_fusion.cc",
        "fusion/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.cc",
        "fusion/data_fusion/tracker/pbf_tracker/pbf_tracker.cc",
        "fusion/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.cc",
        "fusion/dummy/dummy_algorithms.cc",
        "fusion/fusion_system/probabilistic_fusion/probabilistic_fusion.cc",
        "fusion/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.cc",
    ],
    hdrs = [
        "base/base_forward_declaration.h",
        "base/fusion_frame.h",
        "base/fusion_log.h",
        "base/scene.h",
        "base/sensor.h",
        "base/sensor_data_manager.h",
        "base/sensor_frame.h",
        "base/sensor_object.h",
        "base/track.h",
        "base/track_pool_types.h",
        "common/base_filter.h",
        "common/camera_util.h",
        "common/dst_evidence.h",
        "common/information_filter.h",
        "common/kalman_filter.h",
        "fusion/data_association/hm_data_association/chi_squared_cdf_1_0.0500_0.999900.h",
        "fusion/data_association/hm_data_association/chi_squared_cdf_2_0.0500_0.999900.h",
        "fusion/data_association/hm_data_association/hm_tracks_objects_match.h",
        "fusion/data_association/hm_data_association/probabilities.h",
        "fusion/data_association/hm_data_association/projection_cache.h",
        "fusion/data_association/hm_data_association/track_object_distance.h",
        "fusion/data_association/hm_data_association/track_object_similarity.h",
        "fusion/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h",
        "fusion/data_fusion/motion_fusion/kalman_motion_fusion/kalman_motion_fusion.h",
        "fusion/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.h",
        "fusion/data_fusion/tracker/pbf_tracker/pbf_tracker.h",
        "fusion/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.h",
        "fusion/dummy/dummy_algorithms.h",
        "fusion/fusion_system/probabilistic_fusion/probabilistic_fusion.h",
        "fusion/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.h",
        "interface/base_data_association.h",
        "interface/base_existence_fusion.h",
        "interface/base_fusion_system.h",
        "interface/base_gatekeeper.h",
        "interface/base_motion_fusion.h",
        "interface/base_shape_fusion.h",
        "interface/base_tracker.h",
        "interface/base_type_fusion.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/hdmap:apollo_perception_common_hdmap",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/multi_sensor_fusion/proto:dst_existence_fusion_config_cc_proto",
        "//modules/perception/multi_sensor_fusion/proto:dst_type_fusion_config_cc_proto",
        "//modules/perception/multi_sensor_fusion/proto:fusion_component_config_cc_proto",
        "//modules/perception/multi_sensor_fusion/proto:pbf_gatekeeper_config_cc_proto",
        "//modules/perception/multi_sensor_fusion/proto:pbf_tracker_config_cc_proto",
        "//modules/perception/multi_sensor_fusion/proto:probabilistic_fusion_config_cc_proto",
        "@boost",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

apollo_component(
    name = "libmulti_sensor_fusion.so",
    srcs = ["multi_sensor_fusion_component.cc"],
    hdrs = ["multi_sensor_fusion_component.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        ":apollo_perception_multi_sensor_fusion",
        "//modules/common/util:util_tool",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/hdmap:apollo_perception_common_hdmap",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/multi_sensor_fusion/proto:fusion_component_config_cc_proto",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "sensor_frame_test",
    size = "small",
    srcs = ["base/sensor_frame_test.cc"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "camera_util_test",
    size = "small",
    srcs = ["common/camera_util_test.cc"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "dst_evidence_test",
    size = "small",
    srcs = ["common/dst_evidence_test.cc"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "information_filter_test",
    size = "small",
    srcs = ["common/information_filter_test.cc"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "kalman_filter_test",
    size = "small",
    srcs = ["common/kalman_filter_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "hm_data_association_test",
    size = "small",
    srcs = ["fusion/data_association/hm_data_association/hm_data_association_test.cc"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "track_object_distance_test",
    size = "small",
    srcs = ["fusion/data_association/hm_data_association/track_object_distance_test.cc"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "track_object_test",
    size = "small",
    srcs = ["fusion/data_association/hm_data_association/track_object_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "dst_existence_fusion_test",
    size = "small",
    srcs = ["fusion/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion_test.cc"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pbf_shape_fusion_test",
    size = "small",
    srcs = ["fusion/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion_test.cc"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pbf_tracker_test",
    size = "small",
    srcs = ["fusion/data_fusion/tracker/pbf_tracker/pbf_tracker_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "dst_type_fusion_test",
    size = "small",
    srcs = ["fusion/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion_test.cc"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "dummy_algorithms_test",
    size = "small",
    srcs = ["fusion/dummy/dummy_algorithms_test.cc"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "@com_google_googletest//:gtest_main",
    ],
)

# ignore temporarily TODO:// need fix logic

apollo_cc_test(
    name = "probabilistic_fusion_test",
    size = "small",
    srcs = ["fusion/fusion_system/probabilistic_fusion/probabilistic_fusion_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":apollo_perception_multi_sensor_fusion",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()

cpplint()
