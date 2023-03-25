load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PREDICTION_COPTS = ["-DMODULE_NAME=\\\"prediction\\\""]


install(
    name = "prediction_testdata_install",
    data_dest = "prediction/addition_data",
    data = [":prediction_testdata"],
)

cc_library(
    name = "prediction_component_lib",
    srcs = ["prediction_component.cc"],
    hdrs = ["prediction_component.h"],
    copts = PREDICTION_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/prediction/common:message_process",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/prediction/proto:offline_features_cc_proto",
        "//modules/prediction/scenario:scenario_manager",
        "//modules/prediction/submodules:evaluator_submodule_lib",
        "//modules/prediction/submodules:predictor_submodule_lib",
        "//modules/prediction/submodules:submodule_output",
        "//modules/common/util:util_tool",
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
    linkopts = [
        "-lgomp",
    ],
    deps = [
        ":prediction_component_lib",
    ],
    linkstatic = True,
)

cc_binary(
    name = "libprediction_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":prediction_component_lib"],
)

install(
    name = "install",
    library_dest = "prediction/lib",
    data_dest = "prediction",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":prediction.BUILD",
    ],
    targets = [
        ":libprediction_component.so",
    ],
    deps = [
        ":pb_hdrs",
        "//modules/prediction/pipeline:install",
        "//modules/prediction/common:install",
        "//modules/prediction/submodules:install",
        ":prediction_testdata_install",
        "//modules/prediction/proto:py_pb_prediction",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "prediction/include",
    data = [
        "//modules/prediction/proto:fnn_model_base_cc_proto",
        "//modules/prediction/proto:fnn_vehicle_model_cc_proto",
        "//modules/prediction/proto:network_layers_cc_proto",
        "//modules/prediction/proto:network_model_cc_proto",
        "//modules/prediction/proto:offline_features_cc_proto",
        "//modules/prediction/proto:prediction_conf_cc_proto",
        "//modules/prediction/proto:vector_net_cc_proto",
    ],
)
install_src_files(
    name = "install_src",
    deps = [
        ":install_prediction_src",
        ":install_prediction_hdrs"
    ],
)

install_src_files(
    name = "install_prediction_src",
    src_dir = ["."],
    dest = "prediction/src",
    filter = "*",
)

install_src_files(
    name = "install_prediction_hdrs",
    src_dir = ["."],
    dest = "prediction/include",
    filter = "*.h",
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
