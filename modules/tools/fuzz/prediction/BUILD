load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "prediction_fuzz",
    srcs = [
        "prediction_fuzz.cc",
    ],
    copts = [
        "-fsanitize=fuzzer,address,undefined",
        "-Iexternal/libprotobuf_mutator/src/",
    ],
    linkopts = [
        "-fsanitize=fuzzer,address,undefined",
        "-lubsan",
    ],
    deps = [
        "//modules/prediction:prediction_lib",
        "//modules/tools/fuzz/prediction/proto:prediction_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cc_binary(
    name = "predictor_manager_fuzz",
    srcs = ["predictor_manager_fuzz.cc"],
    copts = [
        "-fsanitize=fuzzer,address,undefined",
        "-Iexternal/libprotobuf_mutator/src/",
    ],
    linkopts = [
        "-fsanitize=fuzzer,address,undefined",
        "-lubsan",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/prediction/proto:prediction_conf_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cc_binary(
    name = "evaluator_manager_fuzz",
    srcs = ["evaluator_manager_fuzz.cc"],
    copts = [
        "-fsanitize=fuzzer,address,undefined",
        "-Iexternal/libprotobuf_mutator/src/",
    ],
    linkopts = [
        "-fsanitize=fuzzer,address,undefined",
        "-lubsan",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/evaluator/vehicle:cost_evaluator",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "//modules/prediction/evaluator/vehicle:rnn_evaluator",
        "//modules/prediction/proto:prediction_conf_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cpplint()