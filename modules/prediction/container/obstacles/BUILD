package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacles_container",
    srcs = ["obstacles_container.cc"],
    hdrs = ["obstacles_container.h"],
    deps = [
        "//modules/prediction/container:container",
        "//modules/perception/proto:perception_proto"
    ]
)

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cc"],
    hdrs = ["obstacle.h"],
    deps = [
        "//modules/common/proto:error_code_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/proto:feature_proto",
        "//modules/common:log",
        "//modules/common/math:kalman_filter",
    ],
)

