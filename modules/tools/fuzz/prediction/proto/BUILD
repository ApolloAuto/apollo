package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "prediction_fuzz_proto",
    deps = [
        ":prediction_fuzz_proto_lib",
    ],
)

proto_library(
    name = "prediction_fuzz_proto_lib",
    srcs = [
        "prediction_fuzz.proto",
    ],
    deps = [
        "//modules/localization/proto:localization_proto_lib",
        "//modules/planning/proto:planning_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
    ],
)