load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "prediction_component_lib",
    srcs = ["prediction_component.cc"],
    hdrs = ["prediction_component.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//cyber/common:file",
        "//modules/common/adapters:adapter_gflags",
        "//modules/prediction/common:message_process",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/prediction/proto:offline_features_proto",
        "//modules/prediction/scenario:scenario_manager",
        "//modules/prediction/submodules:evaluator_submodule_lib",
        "//modules/prediction/submodules:predictor_submodule_lib",
        "//modules/prediction/submodules:submodule_output",
        "//modules/prediction/util:data_extraction",
    ],
    alwayslink = True,
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

cpplint()
