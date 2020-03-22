load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "prediction_gflags",
    srcs = ["prediction_gflags.cc"],
    hdrs = ["prediction_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "prediction_system_gflags",
    srcs = ["prediction_system_gflags.cc"],
    hdrs = ["prediction_system_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "prediction_util",
    srcs = ["prediction_util.cc"],
    hdrs = ["prediction_util.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":prediction_gflags",
        ":prediction_map",
    ],
)

cc_test(
    name = "prediction_util_test",
    size = "small",
    srcs = ["prediction_util_test.cc"],
    deps = [
        ":prediction_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "prediction_map",
    srcs = ["prediction_map.cc"],
    hdrs = ["prediction_map.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":prediction_gflags",
        "//modules/map/pnc_map",
    ],
)

cc_test(
    name = "prediction_map_test",
    size = "small",
    srcs = ["prediction_map_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":kml_map_based_test",
        ":prediction_map",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "feature_output",
    srcs = ["feature_output.cc"],
    hdrs = ["feature_output.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/common/util",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/proto:offline_features_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_test(
    name = "feature_output_test",
    size = "small",
    srcs = ["feature_output_test.cc"],
    deps = [
        ":feature_output",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "road_graph",
    srcs = ["road_graph.cc"],
    hdrs = ["road_graph.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":prediction_map",
        "//modules/prediction/common:prediction_constants",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_system_gflags",
        "//modules/prediction/proto:lane_graph_proto",
    ],
)

cc_test(
    name = "road_graph_test",
    size = "small",
    srcs = ["road_graph_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":kml_map_based_test",
        ":road_graph",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "kml_map_based_test",
    hdrs = ["kml_map_based_test.h"],
)

cc_library(
    name = "validation_checker",
    srcs = ["validation_checker.cc"],
    hdrs = ["validation_checker.h"],
    deps = [
        "//modules/common/math",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/proto:lane_graph_proto",
    ],
)

cc_test(
    name = "validation_checker_test",
    size = "small",
    srcs = ["validation_checker_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":validation_checker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "environment_features",
    srcs = ["environment_features.cc"],
    hdrs = ["environment_features.h"],
    deps = [
        "//cyber",
        "//modules/common/proto:geometry_proto",
    ],
)

cc_test(
    name = "environment_features_test",
    size = "small",
    srcs = ["environment_features_test.cc"],
    deps = [
        ":environment_features",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "junction_analyzer",
    srcs = ["junction_analyzer.cc"],
    hdrs = ["junction_analyzer.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":prediction_map",
        "//modules/prediction/proto:feature_proto",
    ],
)

cc_test(
    name = "junction_analyzer_test",
    size = "small",
    srcs = ["junction_analyzer_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":junction_analyzer",
        ":kml_map_based_test",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "message_process",
    srcs = ["message_process.cc"],
    hdrs = ["message_process.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":semantic_map",
        "//cyber/common:file",
        "//modules/common/adapters:adapter_gflags",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/prediction/proto:offline_features_proto",
        "//modules/prediction/scenario:scenario_manager",
        "//modules/prediction/util:data_extraction",
    ],
)

cc_library(
    name = "prediction_thread_pool",
    srcs = ["prediction_thread_pool.cc"],
    hdrs = ["prediction_thread_pool.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//cyber/base",
        "//cyber/common",
    ],
)

cc_test(
    name = "prediction_thread_pool_test",
    size = "small",
    srcs = ["prediction_thread_pool_test.cc"],
    deps = [
        ":prediction_thread_pool",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "semantic_map",
    srcs = ["semantic_map.cc"],
    hdrs = ["semantic_map.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":prediction_gflags",
        "//cyber/common",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/pose:pose_container",
        "//modules/prediction/proto:feature_proto",
        "@opencv",
    ],
)

cc_library(
    name = "prediction_constants",
    hdrs = ["prediction_constants.h"],
)

cpplint()
