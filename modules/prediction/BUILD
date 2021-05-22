load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]

cc_library(
    name = "prediction_component_lib",
    srcs = ["prediction_component.cc"],
    hdrs = ["prediction_component.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//cyber/common:file",
        "//modules/common/adapters:adapter_gflags",
        "//modules/prediction/common:message_process",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/prediction/proto:offline_features_cc_proto",
        "//modules/prediction/scenario:scenario_manager",
        "//modules/prediction/submodules:evaluator_submodule_lib",
        "//modules/prediction/submodules:predictor_submodule_lib",
        "//modules/prediction/submodules:submodule_output",
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
    linkopts = [
        "-lgomp",
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

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libprediction_component.so",
    ],
    deps = [
        "//cyber:install",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = [
        ":prediction_data",
        ":prediction_conf",
    ] + glob([
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

filegroup(
    name = "prediction_data",
    srcs = glob([
        "data/*.pt",
        "data/*.bin",
    ]),
)

filegroup(
    name = "prediction_conf",
    srcs = glob([
        "conf/*.conf",
        "conf/*.txt",
    ]),
)

filegroup(
    name = "prediction_testdata",
    srcs = glob(["testdata/**"]),
)

cpplint()
