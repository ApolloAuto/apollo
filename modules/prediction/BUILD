load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "prediction_component_lib",
    srcs = ["prediction_component.cc"],
    hdrs = [
        "prediction_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//cyber/common:file",
        "//modules/common/adapters:adapter_gflags",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/prediction/proto:offline_features_proto",
        "//modules/prediction/scenario:scenario_manager",
        "//modules/prediction/util:data_extraction",
    ],
)

cc_test(
    name = "prediction_component_test",
    size = "small",
    srcs = ["prediction_component_test.cc"],
    data = [
        ":prediction_conf",
        ":prediction_data",
        ":prediction_testdata",
    ],
    deps = [
        ":prediction_component_lib",
    ],
)

cc_binary(
    name = "libprediction_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":prediction_component_lib"],
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

cc_binary(
    name = "feature_proto_file_to_model_features",
    srcs = ["feature_proto_file_to_model_features.cc"],
    deps = [
        "//modules/common/adapters/proto:adapter_config_proto",
        ":prediction_component_lib",
    ],
)

cc_binary(
    name = "evaluate_prediction_output",
    srcs = ["evaluate_prediction_output.cc"],
    deps = [
        "//cyber/common:file",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/proto:feature_proto",
        "//modules/prediction/proto:offline_features_proto",
        "//modules/prediction/proto:prediction_conf_proto",
        "//modules/prediction/proto:prediction_proto",
        "//modules/prediction/proto:prediction_output_evaluation_proto",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
    ],
)

cpplint()
