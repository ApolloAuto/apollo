cc_library(
    name = "lidar_obstacle_segmentation",
    srcs = [
        "lidar_obstacle_segmentation.cc",
    ],
    hdrs = [
        "lidar_obstacle_segmentation.h",
    ],
    deps = [
        "@eigen",
        "//framework:cybertron",
        "//modules/common/proto:header_proto",
        "//modules/common/proto:error_code_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/lidar/app/proto:app_proto",
        "//modules/perception/lidar/lib/map_manager",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank",
        "//modules/perception/lidar/lib/pointcloud_preprocessor",
        "//modules/perception/lidar/lib/scene_manager",
        "//modules/perception/proto:perception_config_schema_proto",
        "//modules/perception/lib/utils",
    ],
)

cc_library(
    name = "lidar_obstacle_tracking",
    srcs = [
        "lidar_obstacle_tracking.cc",
    ],
    hdrs = [
        "lidar_obstacle_tracking.h",
    ],
    deps = [
        "@eigen",
        "//framework:cybertron",
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/io:protobuf_util",
        "//modules/perception/lib/utils",
        "//modules/perception/lidar/app/proto:app_proto",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
    ],
)
