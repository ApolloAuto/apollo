load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_camera_updater",
    srcs = [
        "perception_camera_updater.cc",
    ],
    hdrs = [
        "perception_camera_updater.h",
    ],
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/common/proto:geometry_proto",
        "//modules/transform/proto:transform_proto",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/proto:camera_update_proto",
        "//third_party/json",
        "@eigen",
    ],
)

cpplint()
