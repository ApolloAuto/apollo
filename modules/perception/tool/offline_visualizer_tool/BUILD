load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "offline_lidar_visualizer_tool",
    srcs = ["offline_lidar_visualizer_tool.cc"],
    data = [
        "//modules/perception:perception_model",
        "//modules/perception/tool/offline_visualizer_tool/conf:config",
    ],
    linkstatic = 0,
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/util",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/obstacle/onboard:lidar_process",
        "@eigen",
        "@glew",
        "@glfw",
        "@opengl",
    ],
)

cc_binary(
    name = "offline_sequential_obstacle_perception_test",
    srcs = ["offline_sequential_obstacle_perception_test.cc"],
    data = [
        "//modules/perception:perception_model",
        "//modules/perception/tool/offline_visualizer_tool/conf:config",
    ],
    linkstatic = 0,
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer",
        "//modules/perception/obstacle/onboard:lidar_process",
        "//modules/perception/obstacle/onboard:obstacle_perception",
        "@eigen",
        "@glew",
        "@glfw",
        "@opengl",
    ],
)

cpplint()
