package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "static_info_proto",
    deps = [":static_info_proto_lib"],
)

proto_library(
    name = "static_info_proto_lib",
    srcs = ["static_info.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/configs/proto:vehicle_config_proto_lib",
        "//modules/control/proto:control_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

proto_library(
    name = "record_proto_lib",
    srcs = ["record.proto"],
    deps = [
        "//cyber/proto:record_proto",
        "//modules/common/proto:drive_event_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/dreamview/proto:hmi_status_proto_lib",
    ],
)

cc_proto_library(
    name = "record_request_proto",
    deps = [":record_request_proto_lib"],
)

proto_library(
    name = "record_request_proto_lib",
    srcs = ["record_request.proto"],
    deps = ["//modules/common/proto:header_proto_lib"],
)

cc_proto_library(
    name = "record_response_proto",
    deps = [":record_response_proto_lib"],
)

proto_library(
    name = "record_response_proto_lib",
    srcs = ["record_response.proto"],
    deps = ["//modules/common/proto:header_proto_lib"],
)

proto_library(
    name = "tools_frame_proto_lib",
    srcs = ["frame.proto"],
)

cc_proto_library(
    name = "data_conf_proto",
    deps = [":data_conf_proto_lib"],
)

proto_library(
    name = "data_conf_proto_lib",
    srcs = ["data_conf.proto"],
)

cc_proto_library(
    name = "data_proto",
    deps = [":data_proto_lib"],
)

proto_library(
    name = "data_proto_lib",
    srcs = ["data.proto"],
    deps = [
        ":record_request_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)
