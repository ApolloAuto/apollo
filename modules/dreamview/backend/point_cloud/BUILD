load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "point_cloud_updater",
    srcs = [
        "point_cloud_updater.cc",
    ],
    hdrs = [
        "point_cloud_updater.h",
    ],
    linkopts = [
        "-lboost_thread",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/proto:point_cloud_proto",
        "//modules/localization/proto:localization_proto",
        "//third_party/json",
        "@com_google_protobuf//:protobuf",
        "@pcl//:pcl",
    ],
)

cpplint()
