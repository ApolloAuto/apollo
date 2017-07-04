package(default_visibility = ["//visibility:public"])

cc_library(
    name = "prediction_lib",
    srcs = ["prediction.cc"],
    hdrs = ["prediction.h"],
    deps = [
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:prediction_common",
        "//modules/prediction/proto:prediction_proto",
    ],
)

cc_binary(
    name = "prediction",
    srcs = ["main.cc"],
    deps = [
        ":prediction_lib",
        "//external:gflags",
    ],
)
