load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "usb_cam_proto",
    deps = [
        ":sensor_usb_cam_proto",
    ],
)


cc_proto_library(
    name = "sensor_usb_cam_proto",
    deps = [
        ":usb_cam_proto_lib",
    ],
)

proto_library(
    name = "usb_cam_proto_lib",
    srcs = [
        "config.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
