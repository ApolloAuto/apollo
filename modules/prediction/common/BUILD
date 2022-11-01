load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])
PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]

cc_library(
    name = "prediction_gflags",
    srcs = ["libprediction_gflags.so"],
    hdrs = ["prediction_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libprediction_gflags.so",
    srcs = ["prediction_gflags.cc", "prediction_gflags.h"],
    linkshared = True,
    linkstatic = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

install(
    name = "install",
    library_dest = "prediction/lib",
    targets = [
        ":libprediction_gflags.so",
        ":libprediction_system_gflags.so"
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libprediction_system_gflags.so",
    srcs = ["prediction_system_gflags.cc", "prediction_system_gflags.h"],
    linkshared = True,
    linkstatic = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "prediction_system_gflags",
    srcs = [":libprediction_system_gflags.so"],
    hdrs = ["prediction_system_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "prediction_util",
    srcs = ["prediction_util.cc"],
    hdrs = ["prediction_util.h"],
    copts = PREDICTION_COPTS,
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
    copts = PREDICTION_COPTS,
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
    copts = PREDICTION_COPTS,
    deps = [
        "//modules/common/util",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/proto:offline_features_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
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
    copts = PREDICTION_COPTS,
    deps = [
        ":prediction_map",
        "//modules/prediction/common:prediction_constants",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_system_gflags",
        "//modules/common_msgs/prediction_msgs:lane_graph_cc_proto",
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
        "//modules/common_msgs/prediction_msgs:lane_graph_cc_proto",
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
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
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
    linkstatic = True,
)

cc_library(
    name = "junction_analyzer",
    srcs = ["junction_analyzer.cc"],
    hdrs = ["junction_analyzer.h"],
    copts = PREDICTION_COPTS,
    deps = [
        ":prediction_map",
        "//modules/common_msgs/prediction_msgs:feature_cc_proto",
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
    copts = PREDICTION_COPTS,
    deps = [
        ":semantic_map",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/prediction/proto:offline_features_cc_proto",
        "//modules/prediction/scenario:scenario_manager",
        "//modules/common/util:util_tool",
    ],
)

cc_library(
    name = "prediction_thread_pool",
    srcs = ["prediction_thread_pool.cc"],
    hdrs = ["prediction_thread_pool.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//cyber",
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
    copts = PREDICTION_COPTS,
    deps = [
        ":prediction_gflags",
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/pose:pose_container",
        "//modules/common_msgs/prediction_msgs:feature_cc_proto",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
    ],
)

cc_library(
    name = "prediction_constants",
    hdrs = ["prediction_constants.h"],
)

cpplint()
