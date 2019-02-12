load("//tools:cpplint.bzl", "cpplint")

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
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_model/proto:vehicle_model_config_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_proto",
    ],
)

cpplint()
