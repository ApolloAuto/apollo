load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_model",
    srcs = ["vehicle_model.cc"],
    hdrs = ["vehicle_model.h"],
    deps = [
        "//cyber/common:file",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_model/proto:vehicle_model_config_cc_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
    ],
)

cc_test(
    name = "vehicle_model_test",
    size = "small",
    srcs = ["vehicle_model_test.cc"],
    data = [
        ":test_data",
    ],
    deps = [
        ":vehicle_model",
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

install(
    name = "install",
    data = [
        ":vehicle_model_config_data",
    ],
)

cpplint()
