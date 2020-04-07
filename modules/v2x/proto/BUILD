load("//tools:py_proto.bzl", "py_proto")
load("@com_github_grpc_grpc//bazel:cc_grpc_library.bzl", "cc_grpc_library")

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
    srcs = [":v2x_service_car_to_obu_proto_lib"],
    deps = [":v2x_service_car_to_obu_proto"],
    grpc_only = True,
)

cc_proto_library(
    name = "v2x_service_car_to_obu_proto",
    deps = [":v2x_service_car_to_obu_proto_lib"],
)

proto_library(
    name = "v2x_service_car_to_obu_proto_lib",
    srcs = ["v2x_service_car_to_obu.proto"],
    deps = [
        ":v2x_carstatus_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
    ],
)

cc_grpc_library(
    name = "v2x_service_obu_to_car_grpc",
    srcs = [":v2x_service_obu_to_car_proto_lib"],
    deps = [":v2x_service_obu_to_car_proto"],
    grpc_only = True,
)

cc_proto_library(
    name = "v2x_service_obu_to_car_proto",
    deps = [":v2x_service_obu_to_car_proto_lib"],
)

proto_library(
    name = "v2x_service_obu_to_car_proto_lib",
    srcs = ["v2x_service_obu_to_car.proto"],
    deps = [
        ":v2x_traffic_light_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
    ],
)

py_proto(
    name = "v2x_service_obu_to_car_pb2",
    src = "v2x_service_obu_to_car.proto",
    deps = [
        "//modules/perception/proto:perception_obstacle_pb2",
        "//modules/v2x/proto:v2x_traffic_light_pb2",
    ],
)

py_proto(
    name = "v2x_carstatus_pb2",
    src = "v2x_carstatus.proto",
    deps = [
        "//modules/canbus/proto:chassis_detail_pb2",
        "//modules/localization/proto:localization_pb2",
    ],
)

py_proto(
    name = "v2x_service_car_to_obu_pb2",
    src = "v2x_service_car_to_obu.proto",
    deps = [
        "//modules/perception/proto:perception_obstacle_pb2",
        "//modules/v2x/proto:v2x_carstatus_pb2",
    ],
)

py_proto(
    name = "v2x_traffic_light_pb2",
    src = "v2x_traffic_light.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)
