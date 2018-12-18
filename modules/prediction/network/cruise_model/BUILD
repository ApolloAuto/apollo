load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cruise_model",
    srcs = [
        "cruise_model.cc",
    ],
    hdrs = [
        "cruise_model.h",
    ],
    deps = [
        "//cyber",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/network:net_layer",
        "//modules/prediction/network:net_model",
        "//modules/prediction/proto:cruise_model_proto",
        "@eigen",
    ],
)

cpplint()
