load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "offline_lidar_visualizer_tool",
    srcs = ["offline_lidar_visualizer_tool.cc"],
    data = [
        "//modules/perception:perception_model",
        "//modules/perception/tool/offline_visualizer_tool/conf:perception_tool_config",
    ],
    linkstatic = 0,
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer:perception_obstacle_lidar_opengl_visualizer",
        "//modules/perception/obstacle/onboard:perception_obstacle_lidar_process",
        "@eigen//:eigen",
        "@glew//:glew",
        "@glfw//:glfw",
        "@opengl//:opengl",
    ],
)

cc_binary(
    name = "offline_sequential_obstacle_perception_test",
    srcs = ["offline_sequential_obstacle_perception_test.cc"],
    data = [
        "//modules/perception:perception_model",
        "//modules/perception/tool/offline_visualizer_tool/conf:perception_tool_config",
    ],
    linkstatic = 0,
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common:perception_common",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/onboard:perception_obstacle_obstacle_perception",
        "//modules/perception/obstacle/lidar/visualizer/opengl_visualizer:perception_obstacle_lidar_opengl_visualizer",
        "//modules/perception/obstacle/onboard:perception_obstacle_lidar_process",
        "@eigen//:eigen",
        "@glew//:glew",
        "@glfw//:glfw",
        "@opengl//:opengl",
    ],
)

cpplint()
