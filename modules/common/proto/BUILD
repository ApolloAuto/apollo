package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "error_code_proto",
    deps = [
        ":error_code_proto_lib",
    ],
)

proto_library(
    name = "error_code_proto_lib",
    srcs = [
        "error_code.proto",
    ],
)

cc_proto_library(
    name = "header_proto",
    deps = [
        ":header_proto_lib",
    ],
)

proto_library(
    name = "header_proto_lib",
    srcs = [
        "header.proto",
    ],
    deps = [
        ":error_code_proto_lib",
    ],
)

cc_proto_library(
    name = "vehicle_signal_proto",
    deps = [
        ":vehicle_signal_proto_lib",
    ],
)

proto_library(
    name = "vehicle_signal_proto_lib",
    srcs = [
        "vehicle_signal.proto",
    ],
)

cc_proto_library(
    name = "common_proto",
    deps = [
        ":common_proto_lib",
    ],
)

proto_library(
    name = "common_proto_lib",
    srcs = [
        "geometry.proto",
    ],
    deps = [
        ":error_code_proto_lib",
        ":header_proto_lib",
    ],
)

cc_proto_library(
    name = "gnss_status_proto",
    deps = [
        ":gnss_status_proto_lib",
    ],
)

proto_library(
    name = "gnss_status_proto_lib",
    srcs = [
        "gnss_status.proto",
    ],
    deps = [
        ":error_code_proto_lib",
        ":header_proto_lib",
    ],
)

cc_proto_library(
    name = "pnc_point_proto",
    deps = [
        ":pnc_point_proto_lib",
    ],
)

proto_library(
    name = "pnc_point_proto_lib",
    srcs = [
        "pnc_point.proto",
    ],
)

cc_proto_library(
    name = "vehicle_state_proto",
    deps = [
        ":vehicle_state_proto_lib",
    ],
)

proto_library(
    name = "vehicle_state_proto_lib",
    srcs = [
        "vehicle_state.proto",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/localization/proto:pose_proto_lib",
    ],
)

cc_proto_library(
    name = "car_sound_proto",
    deps = [
        ":car_sound_proto_lib",
    ],
)

proto_library(
    name = "car_sound_proto_lib",
    srcs = [
        "car_sound.proto",
    ],
    deps = [
        ":header_proto_lib",
    ],
)

cc_proto_library(
    name = "drive_event_proto",
    deps = [
        ":drive_event_proto_lib",
    ],
)

proto_library(
    name = "drive_event_proto_lib",
    srcs = [
        "drive_event.proto",
    ],
    deps = [
        ":header_proto_lib",
    ],
)
