load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "perception_traffic_light_visualizer",
    srcs = [
        "tl_visualizer.cc",
    ],

    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/adapters:adapter_gflags",
        "//modules/perception/traffic_light/base:perception_traffic_light_base",
        "//modules/perception/traffic_light/util:perception_traffic_light_util",
        "//modules/perception/lib/base",
        "//modules/perception/common:perception_common",
        "//modules/perception/proto:perception_proto",
        "@ros//:ros_common",
    ],
    linkopts = ["-lopencv_core -lopencv_imgproc -lopencv_highgui"],
)

cpplint()
