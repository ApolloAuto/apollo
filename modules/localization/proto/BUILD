load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "localization_config_proto",
    deps = [
        ":localization_config_proto_lib",
    ],
)

proto_library(
    name = "localization_config_proto_lib",
    srcs = ["localization_config.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
    ],
)

cc_proto_library(
    name = "pose_proto",
    deps = [
        ":pose_proto_lib",
    ],
)

proto_library(
    name = "pose_proto_lib",
    srcs = ["pose.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
    ],
)

cc_proto_library(
    name = "gps_proto",
    deps = [
        ":gps_proto_lib",
    ],
)

proto_library(
    name = "gps_proto_lib",
    srcs = ["gps.proto"],
    deps = [
        ":pose_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "imu_proto",
    deps = [
        ":imu_proto_lib",
    ],
)

proto_library(
    name = "imu_proto_lib",
    srcs = ["imu.proto"],
    deps = [
        ":pose_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "localization_proto",
    deps = [
        ":localization_proto_lib",
    ],
)

proto_library(
    name = "localization_proto_lib",
    srcs = ["localization.proto"],
    deps = [
        ":localization_status_proto_lib",
        ":pose_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

cc_proto_library(
    name = "measure_proto",
    deps = [
        ":measure_proto_lib",
    ],
)

proto_library(
    name = "measure_proto_lib",
    srcs = ["measure.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "sins_pva_proto",
    deps = [
        ":sins_pva_proto_lib",
    ],
)

proto_library(
    name = "sins_pva_proto_lib",
    srcs = ["sins_pva.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "gnss_pnt_result_proto",
    deps = [
        ":gnss_pnt_result_proto_lib",
    ],
)

proto_library(
    name = "gnss_pnt_result_proto_lib",
    srcs = ["gnss_pnt_result.proto"],
    deps = [
        "//modules/drivers/gnss/proto:gnss_raw_observation_lib",
    ],
)

cc_proto_library(
    name = "localization_status_proto",
    deps = [
        ":localization_status_proto_lib",
    ],
)

proto_library(
    name = "localization_status_proto_lib",
    srcs = ["localization_status.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "rtk_config_proto",
    deps = [
        ":rtk_config_proto_lib",
    ],
)

proto_library(
    name = "rtk_config_proto_lib",
    srcs = ["rtk_config.proto"],
)

cpplint()
