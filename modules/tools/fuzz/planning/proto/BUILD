package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "navi_planning_fuzz_proto",
    deps = [
        ":navi_planning_fuzz_proto_lib",
    ],
)

proto_library(
    name = "navi_planning_fuzz_proto_lib",
    srcs = [
        "navi_planning_fuzz.proto",
    ],
    deps = [
        "//modules/localization/proto:localization_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
        "//modules/prediction/proto:prediction_proto_lib",
    ],
)

cc_proto_library(
    name = "std_planning_fuzz_proto",
    deps = [
        ":std_planning_fuzz_proto_lib",
    ],
)

proto_library(
    name = "std_planning_fuzz_proto_lib",
    srcs = [
        "std_planning_fuzz.proto",
    ],
    deps = [
        "//modules/localization/proto:localization_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
        "//modules/prediction/proto:prediction_proto_lib",
    ],
)