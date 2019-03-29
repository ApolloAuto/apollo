load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "feature_proto_file_to_model_features",
    srcs = ["feature_proto_file_to_model_features.cc"],
    deps = [
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/prediction:prediction_component_lib",
    ],
)

cc_binary(
    name = "evaluate_prediction_output",
    srcs = ["evaluate_prediction_output.cc"],
    deps = [
        "//cyber/common:file",
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/prediction/proto:feature_proto",
        "//modules/prediction/proto:offline_features_proto",
        "//modules/prediction/proto:prediction_conf_proto",
        "//modules/prediction/proto:prediction_output_evaluation_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_binary(
    name = "records_to_offline_data",
    srcs = ["records_to_offline_data.cc"],
    deps = [
        "//modules/prediction/common:message_process",
    ],
    linkopts = [
        "-lboost_filesystem",
    ],
)

cpplint()
