load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "perception_traffic_light_visualizer",
    srcs = [
        "tl_visualizer.cc",
    ],
    linkopts = ["-lopencv_core -lopencv_imgproc -lopencv_highgui"],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/adapters:adapter_gflags",
        "//modules/perception/common",
        "//modules/perception/lib/base",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/traffic_light/base",
        "//modules/perception/traffic_light/util",
        "@ros//:ros_common",
    ],
)

cpplint()
