package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "rtk_localization_fuzz_proto",
    deps = [
        ":rtk_localization_fuzz_proto_lib",
    ],
)

proto_library(
    name = "rtk_localization_fuzz_proto_lib",
    srcs = [
        "rtk_localization_fuzz.proto",
    ],
    deps = [
        "//modules/localization/proto:imu_proto_lib",
        "//modules/localization/proto:gps_proto_lib",
    ],
)

cc_proto_library(
    name = "msf_localization_fuzz_proto",
    deps = [
        ":msf_localization_fuzz_proto_lib",
    ],
)

proto_library(
    name = "msf_localization_fuzz_proto_lib",
    srcs = [
        "msf_localization_fuzz.proto",
    ],
    deps = [
        "//modules/drivers/gnss/proto:imu_lib",
        "//modules/drivers/gnss/proto:gnss_best_pose_lib",
        "//modules/drivers/gnss/proto:gnss_raw_observation_lib",
    ],
)