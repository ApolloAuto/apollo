load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "prediction_lib",
    srcs = ["prediction.cc"],
    hdrs = [
        "prediction.h",
        "prediction_interface.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/common/math:geometry",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/common:validation_checker",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/prediction/proto:prediction_conf_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_test(
    name = "prediction_test",
    size = "small",
    srcs = ["prediction_test.cc"],
    data = [
        ":prediction_conf",
        ":prediction_data",
        ":prediction_testdata",
    ],
    deps = [
        ":prediction_lib",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "prediction",
    srcs = ["main.cc"],
    deps = [
        ":prediction_lib",
        "//external:gflags",
    ],
)

filegroup(
    name = "prediction_data",
    srcs = glob(["data/**"]),
)

filegroup(
    name = "prediction_conf",
    srcs = glob(["conf/**"]),
)

filegroup(
    name = "prediction_testdata",
    srcs = glob(["testdata/**"]),
)

cpplint()
