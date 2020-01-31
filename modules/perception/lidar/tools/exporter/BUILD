load("//tools:cpplint.bzl", "cpplint")

cc_library(
    name = "msgexporterlib",
    srcs = ["msg_exporter.cc"],
    hdrs = ["msg_exporter.h"],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/lidar/common:pcl_util",
        "//modules/perception/onboard/transform_wrapper",
        "//modules/transform/proto:transform_proto",
        "//third_party:opencv",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "msg_exporter",
    srcs = ["msg_exporter_main.cc"],
    deps = [
        ":msgexporterlib",
        "@com_google_absl//absl/strings",
    ],
)

cpplint()
