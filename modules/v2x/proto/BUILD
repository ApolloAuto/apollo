load("//tools:cpplint.bzl", "cpplint")
load("//tools:grpc_library.bzl", "cc_grpc_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "v2x_traffic_light_proto",
    deps = [
        ":v2x_traffic_light_proto_lib",
    ],
)

proto_library(
    name = "v2x_traffic_light_proto_lib",
    srcs = ["v2x_traffic_light.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "v2x_carstatus_proto",
    deps = [
        ":v2x_carstatus_proto_lib",
    ],
)

proto_library(
    name = "v2x_carstatus_proto_lib",
    srcs = ["v2x_carstatus.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/localization/proto:localization_proto_lib",
    ],
)

cc_grpc_library(
    name = "v2x_service_car_to_obu_grpc",
    srcs = ["v2x_service_car_to_obu.proto"],
    deps_cc = [
        ":v2x_carstatus_proto",
        "//modules/perception/proto:perception_proto",
    ],
    deps = [
        ":v2x_carstatus_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
    ],
)

cc_grpc_library(
    name = "v2x_service_obu_to_car_grpc",
    srcs = ["v2x_service_obu_to_car.proto"],
    deps_cc = [
        ":v2x_traffic_light_proto",
        "//modules/perception/proto:perception_proto",
    ],
    deps = [
        ":v2x_traffic_light_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
    ],
)

cpplint()
