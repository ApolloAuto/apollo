load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "testdata",
    srcs = glob(["local_map/test_data/**"]),
)

apollo_cc_library(
    name = "apollo_localization_msf",
    srcs = [
        "common/io/velodyne_utility.cc",
        "common/util/compression.cc",
        "common/util/file_utility.cc",
        "common/util/frame_transform.cc",
        "common/util/system_utility.cc",
        "local_map/base_map/base_map.cc",
        "local_map/base_map/base_map_config.cc",
        "local_map/base_map/base_map_matrix.cc",
        "local_map/base_map/base_map_node.cc",
        "local_map/base_map/base_map_node_index.cc",
        "local_map/base_map/base_map_pool.cc",
        "local_map/lossless_map/lossless_map.cc",
        "local_map/lossless_map/lossless_map_config.cc",
        "local_map/lossless_map/lossless_map_matrix.cc",
        "local_map/lossless_map/lossless_map_node.cc",
        "local_map/lossless_map/lossless_map_pool.cc",
        "local_map/lossy_map/lossy_map_2d.cc",
        "local_map/lossy_map/lossy_map_config_2d.cc",
        "local_map/lossy_map/lossy_map_matrix_2d.cc",
        "local_map/lossy_map/lossy_map_pool_2d.cc",
        "local_map/ndt_map/ndt_map.cc",
        "local_map/ndt_map/ndt_map_config.cc",
        "local_map/ndt_map/ndt_map_matrix.cc",
        "local_map/ndt_map/ndt_map_node.cc",
        "local_map/ndt_map/ndt_map_pool.cc",
        "local_pyramid_map/base_map/base_map.cc",
        "local_pyramid_map/base_map/base_map_config.cc",
        "local_pyramid_map/base_map/base_map_matrix.cc",
        "local_pyramid_map/base_map/base_map_node.cc",
        "local_pyramid_map/base_map/base_map_node_config.cc",
        "local_pyramid_map/base_map/base_map_node_index.cc",
        "local_pyramid_map/base_map/base_map_pool.cc",
        "local_pyramid_map/ndt_map/ndt_map.cc",
        "local_pyramid_map/ndt_map/ndt_map_config.cc",
        "local_pyramid_map/ndt_map/ndt_map_matrix.cc",
        "local_pyramid_map/ndt_map/ndt_map_matrix_handler.cc",
        "local_pyramid_map/ndt_map/ndt_map_node.cc",
        "local_pyramid_map/ndt_map/ndt_map_node_config.cc",
        "local_pyramid_map/ndt_map/ndt_map_pool.cc",
        "local_pyramid_map/pyramid_map/pyramid_map.cc",
        "local_pyramid_map/pyramid_map/pyramid_map_config.cc",
        "local_pyramid_map/pyramid_map/pyramid_map_matrix.cc",
        "local_pyramid_map/pyramid_map/pyramid_map_matrix_handler.cc",
        "local_pyramid_map/pyramid_map/pyramid_map_node.cc",
        "local_pyramid_map/pyramid_map/pyramid_map_node_config.cc",
        "local_pyramid_map/pyramid_map/pyramid_map_pool.cc",
        "local_tool/data_extraction/cyber_record_reader.cc",
        "local_tool/data_extraction/location_exporter.cc",
        "local_tool/data_extraction/pcd_exporter.cc",
        "local_tool/local_visualization/engine/visualization_engine.cc",
        "local_tool/local_visualization/engine/visualization_manager.cc",
        "local_tool/local_visualization/offline_visual/offline_local_visualizer.cc",
        "local_tool/map_creation/poses_interpolation/poses_interpolation.cc",
    ] + glob(["local_integ/*.cc"]),
    hdrs = [
        "common/io/velodyne_utility.h",
        "common/io/pcl_point_types.h",
        "common/util/base_map_cache.h",
        "common/util/compression.h",
        "common/util/extract_ground_plane.h",
        "common/util/file_utility.h",
        "common/util/frame_transform.h",
        "common/util/math_util.h",
        "common/util/rect2d.h",
        "common/util/system_utility.h",
        "common/util/time_conversion.h",
        "common/util/voxel_grid_covariance_hdmap.h",
        "local_map/base_map/base_map.h",
        "local_map/base_map/base_map_fwd.h",
        "local_map/base_map/base_map_config.h",
        "local_map/base_map/base_map_matrix.h",
        "local_map/base_map/base_map_node.h",
        "local_map/base_map/base_map_node_index.h",
        "local_map/base_map/base_map_pool.h",
        "local_map/ndt_map/ndt_map.h",
        "local_map/ndt_map/ndt_map_config.h",
        "local_map/ndt_map/ndt_map_matrix.h",
        "local_map/ndt_map/ndt_map_node.h",
        "local_map/ndt_map/ndt_map_pool.h",
        "local_map/lossless_map/lossless_map.h",
        "local_map/lossless_map/lossless_map_config.h",
        "local_map/lossless_map/lossless_map_matrix.h",
        "local_map/lossless_map/lossless_map_node.h",
        "local_map/lossless_map/lossless_map_pool.h",
        "local_map/lossy_map/lossy_map_2d.h",
        "local_map/lossy_map/lossy_map_config_2d.h",
        "local_map/lossy_map/lossy_map_matrix_2d.h",
        "local_map/lossy_map/lossy_map_pool_2d.h",
        "local_map/lossy_map/lossy_map_node_2d.h",
        "local_pyramid_map/base_map/base_map.h",
        "local_pyramid_map/base_map/base_map_config.h",
        "local_pyramid_map/base_map/base_map_fwd.h",
        "local_pyramid_map/base_map/base_map_matrix.h",
        "local_pyramid_map/base_map/base_map_matrix_handler.h",
        "local_pyramid_map/base_map/base_map_node.h",
        "local_pyramid_map/base_map/base_map_node_config.h",
        "local_pyramid_map/base_map/base_map_node_index.h",
        "local_pyramid_map/base_map/base_map_pool.h",
        "local_pyramid_map/ndt_map/ndt_map.h",
        "local_pyramid_map/ndt_map/ndt_map_config.h",
        "local_pyramid_map/ndt_map/ndt_map_matrix.h",
        "local_pyramid_map/ndt_map/ndt_map_matrix_handler.h",
        "local_pyramid_map/ndt_map/ndt_map_node.h",
        "local_pyramid_map/ndt_map/ndt_map_node_config.h",
        "local_pyramid_map/ndt_map/ndt_map_pool.h",
        "local_pyramid_map/pyramid_map/aligned_matrix.h",
        "local_pyramid_map/pyramid_map/pyramid_map.h",
        "local_pyramid_map/pyramid_map/pyramid_map_config.h",
        "local_pyramid_map/pyramid_map/pyramid_map_matrix.h",
        "local_pyramid_map/pyramid_map/pyramid_map_matrix_handler.h",
        "local_pyramid_map/pyramid_map/pyramid_map_node.h",
        "local_pyramid_map/pyramid_map/pyramid_map_node_config.h",
        "local_pyramid_map/pyramid_map/pyramid_map_pool.h",
        "local_tool/data_extraction/cyber_record_reader.h",
        "local_tool/data_extraction/location_exporter.h",
        "local_tool/data_extraction/pcd_exporter.h",
        "local_tool/local_visualization/engine/visualization_engine.h",
        "local_tool/local_visualization/engine/visualization_manager.h",
        "local_tool/local_visualization/offline_visual/offline_local_visualizer.h",
        "local_tool/map_creation/poses_interpolation/poses_interpolation.h",
    ] + glob(["local_integ/*.h"]),
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_status_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_raw_observation_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/proto:gnss_pnt_result_cc_proto",
        "//modules/localization/proto:localization_config_cc_proto",
        "//modules/localization/proto:measure_cc_proto",
        "//modules/localization/proto:sins_pva_cc_proto",
        "@boost",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@fastrtps",
        "@local_config_pcl//:pcl",
        "@localization_msf",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@proj",
        "@zlib",
    ],
)

apollo_component(
    name = "libmsf_localization_component.so",
    srcs = [
        "msf_localization.cc",
        "msf_localization_component.cc",
    ],
    hdrs = [
        "msf_localization.h",
        "msf_localization_component.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"localization\\\"",
    ],
    deps = [
        ":apollo_localization_msf",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_status_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_cc_proto",
        "//modules/common_msgs/sensor_msgs:heading_cc_proto",
        "//modules/common_msgs/sensor_msgs:imu_cc_proto",
        "//modules/localization/common:gnss_compensator",
        "//modules/localization/common:localization_gflags",
        "//modules/localization/proto:gnss_pnt_result_cc_proto",
        "//modules/localization/proto:localization_config_cc_proto",
        "//modules/localization/proto:measure_cc_proto",
        "//modules/localization/proto:sins_pva_cc_proto",
        "//modules/transform:apollo_transform",
        "@boost",
        "@com_github_google_glog//:glog",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest",
    ],
)

apollo_cc_test(
    name = "msf_localization_test",
    size = "enormous",
    srcs = ["msf_localization_test.cc"],
    copts = [
        "-fno-access-control",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/localization/msf:DO_NOT_IMPORT_msf_localization_component",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pyramid_map_config_test",
    size = "medium",
    timeout = "short",
    srcs = ["local_pyramid_map/pyramid_map/pyramid_map_config_test.cc"],
    deps = [
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pyramid_map_matrix_handler_test",
    size = "medium",
    timeout = "short",
    srcs = ["local_pyramid_map/pyramid_map/pyramid_map_matrix_handler_test.cc"],
    deps = [
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pyramid_map_matrix_test",
    size = "medium",
    timeout = "short",
    srcs = ["local_pyramid_map/pyramid_map/pyramid_map_matrix_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pyramid_map_node_test",
    size = "medium",
    timeout = "short",
    srcs = ["local_pyramid_map/pyramid_map/pyramid_map_node_test.cc"],
    deps = [
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pyramid_map_pool_test",
    size = "large",
    timeout = "short",
    srcs = ["local_pyramid_map/pyramid_map/pyramid_map_pool_test.cc"],
    deps = [
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "pyramid_map_test",
    size = "large",
    timeout = "short",
    srcs = ["local_pyramid_map/pyramid_map/pyramid_map_test.cc"],
    deps = [
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "map_pool_test",
    size = "small",
    srcs = ["local_map/lossless_map/map_pool_test.cc"],
    copts = [
        "-fno-access-control",
    ],
    deps = [
        "//cyber",
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "map_node_index_test",
    timeout = "short",
    srcs = ["local_map/base_map/map_node_index_test.cc"],
    deps = [
        ":apollo_localization_msf",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "lossless_map_config_test",
    size = "small",
    srcs = ["local_map/lossless_map/lossless_map_config_test.cc"],
    data = [":testdata"],
    deps = [
        "//cyber",
        ":apollo_localization_msf",  
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "lossy_map_config_2d_test",
    size = "medium",
    timeout = "short",
    srcs = ["local_map/lossy_map/lossy_map_config_2d_test.cc"],
    data = [
        ":testdata",
    ],
    deps = [
        "//cyber",
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ndt_map_test",
    size = "small",
    timeout = "short",
    srcs = ["local_pyramid_map/ndt_map/ndt_map_test.cc"],
    data = [":testdata"],
    deps = [
        "//cyber",
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ndt_map_config_test",
    size = "small",
    timeout = "short",
    srcs = ["local_pyramid_map/ndt_map/ndt_map_config_test.cc"],
    deps = [
        "//cyber",
        ":apollo_localization_msf", 
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "local_map_ndt_map_test",
    size = "small",
    timeout = "short",
    srcs = ["local_map/ndt_map/ndt_map_test.cc"],
    data = [":testdata"],
    deps = [
        ":apollo_localization_msf",
        "//cyber",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "local_map_ndt_map_config_test",
    size = "small",
    timeout = "short",
    srcs = ["local_map/ndt_map/ndt_map_config_test.cc"],
    deps = [
        ":apollo_localization_msf",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_binary(
    name = "lossless_map_creator",
    srcs = ["local_tool/map_creation/lossless_map_creator.cc"],
    deps = [
        "//cyber",
        ":apollo_localization_msf",
        "@boost",
        "@com_google_absl//:absl",
    ],
)

apollo_cc_binary(
    name = "lossless_map_to_lossy_map",
    srcs = ["local_tool/map_creation/lossless_map_to_lossy_map.cc"],
    deps = [
        ":apollo_localization_msf",
        "@boost",
    ],
)

apollo_cc_binary(
    name = "poses_interpolator",
    srcs = ["local_tool/map_creation/poses_interpolator.cc"],
    deps = [
        ":apollo_localization_msf",
    ],
)

apollo_cc_binary(
    name = "offline_local_visualizer",
    srcs = ["local_tool/local_visualization/offline_visual/offline_local_visualizer_main.cc"],
    deps = [
        ":apollo_localization_msf",
        "@boost",
    ],
)

apollo_cc_binary(
    name = "cyber_record_parser",
    srcs = ["local_tool/data_extraction/cyber_record_parser.cc"],
    deps = [
        ":apollo_localization_msf",
        "//cyber",
        "@boost",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

apollo_cc_binary(
    name = "compare_poses",
    srcs = ["local_tool/data_extraction/compare_poses.cc"],
    deps = [
        ":apollo_localization_msf",
        "//cyber",
        "@boost",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "compression_test",
    size = "small",
    timeout = "short",
    srcs = ["common/util/compression_test.cc"],
    deps = [
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "rect2d_test",
    size = "small",
    timeout = "short",
    srcs = ["common/util/rect2d_test.cc"],
    deps = [
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

filegroup(
    name = "common_test_data",
    srcs = glob(["common/test_data/**"]),
)

apollo_cc_test(
    name = "system_utility_test",
    size = "medium",
    timeout = "short",
    srcs = ["common/util/system_utility_test.cc"],
    data = [":common_test_data"],
    deps = [
        ":apollo_localization_msf",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "frame_transform_test",
    size = "medium",
    timeout = "short",
    srcs = ["common/util/frame_transform_test.cc"],
    deps = [
        ":apollo_localization_msf",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "base_map_cache_test",
    srcs = ["common/util/base_map_cache_test.cc"],
    deps = [
        ":apollo_localization_msf",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()

cpplint()
