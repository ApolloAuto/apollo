load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_test")

filegroup(
    name = "vehicle_model_test_data",
    srcs = glob([
        "testdata/*",
    ]),
)

apollo_cc_library(
    name = "vehicle_model",
    srcs = ["vehicle_model.cc"],
    hdrs = ["vehicle_model.h"],
    deps = [
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_model/proto:vehicle_model_config_cc_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
    ],
    visibility = ["//visibility:public"],
)

apollo_cc_test(
    name = "vehicle_model_test",
    size = "small",
    srcs = ["vehicle_model_test.cc"],
    data = [
        ":test_data",
    ],
    deps = [
        ":vehicle_model",
        "//cyber",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob([
        "testdata/*.txt",
    ]),
)

filegroup(
    name = "vehicle_model_config_data",
    srcs = ["conf/vehicle_model_config.pb.txt"],
)

apollo_package()
cpplint()
