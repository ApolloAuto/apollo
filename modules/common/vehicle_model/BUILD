load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_model",
    srcs = [
        "vehicle_model.cc",
    ],
    hdrs = [
        "vehicle_model.h",
    ],
    deps = [
        "//cyber/common:file",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_model/proto:vehicle_model_config_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_proto",
    ],
)

cc_test(
    name = "vehicle_model_test",
    size = "small",
    srcs = [
        "vehicle_model_test.cc",
    ],
    data = [
        "//modules/common/testdata:vehicle_model_testdata",
    ],
    deps = [
        ":vehicle_model",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "@gtest//:main",
    ],
)

cpplint()
