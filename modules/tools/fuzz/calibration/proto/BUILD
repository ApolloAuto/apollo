package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "lidar_ex_checker_fuzz_proto",
    deps = [
        ":lidar_ex_checker_fuzz_proto_lib",
    ],
)

proto_library(
    name = "lidar_ex_checker_fuzz_proto_lib",
    srcs = [
        "lidar_ex_checker_fuzz.proto",
    ],
    deps = [
        "//modules/localization/proto:gps_proto_lib",
        "//modules/drivers/gnss/proto:ins_proto_lib",
    ],
)

cc_proto_library(
    name = "republish_msg_fuzz_proto",
    deps = [
        ":republish_msg_fuzz_proto_lib",
    ],
)

proto_library(
    name = "republish_msg_fuzz_proto_lib",
    srcs = [
        "republish_msg_fuzz.proto",
    ],
    deps = [
        "//modules/localization/proto:gps_proto_lib",
        "//modules/drivers/gnss/proto:ins_proto_lib",
    ],
)