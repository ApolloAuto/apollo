load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_drivers_lidar_src",
#         ":install_drivers_lidar_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_drivers_lidar_src",
#     src_dir = ["."],
#     dest = "drivers-lidar/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_drivers_lidar_hdrs",
#     src_dir = ["."],
#     dest = "drivers-lidar/include",
#     filter = "*.h",
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "drivers-lidar/include",
#     data = [
#         "//modules/drivers/lidar/proto:config_cc_proto",
#         "//modules/drivers/lidar/proto:hesai_config_cc_proto",
#         "//modules/drivers/lidar/proto:hesai_cc_proto",
#         "//modules/drivers/lidar/proto:lidar_parameter_cc_proto",
#         "//modules/drivers/lidar/proto:robosense_config_cc_proto",
#         "//modules/drivers/lidar/proto:robosense_cc_proto",
#         "//modules/drivers/lidar/proto:velodyne_config_cc_proto",
#         "//modules/drivers/lidar/proto:velodyne_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_drivers_camera_py",
#     dest = "drivers-lidar/python/modules/drivers/lidar",
#     files = [
#         "//modules/drivers/lidar/proto:config_py_pb2",
#         "//modules/drivers/lidar/proto:hesai_config_py_pb2",
#         "//modules/drivers/lidar/proto:hesai_py_pb2",
#         "//modules/drivers/lidar/proto:lidar_parameter_py_pb2",
#         "//modules/drivers/lidar/proto:robosense_config_py_pb2",
#         "//modules/drivers/lidar/proto:robosense_py_pb2",
#         "//modules/drivers/lidar/proto:velodyne_py_pb2",
#         "//modules/drivers/lidar/proto:velodyne_config_py_pb2",
#     ],
# )

filegroup(
    name = "package_data",
    srcs = [
        ":cyberfile.xml",
        ":drivers-lidar.BUILD",
    ],
)

# install(
#     name = "install_package_data",
#     data_dest = "drivers-lidar",
#     data = ["package_data"],
# )

# install(
#     name = "install",
#     data_dest = "drivers-lidar/addition_data",
#     library_dest = "drivers-lidar/lib",
#     data = [
#         ":runtime_data",
#     ],
#     targets = [
#         "liblidar_driver_component.so",
#     ],
#     deps = [
#         ":install_package_data",
#         ":pb_hdrs",
#         ":pb_drivers_camera_py",
#         "//modules/drivers/lidar/hesai:install",
#         "//modules/drivers/lidar/robosense:install",
#         "//modules/drivers/lidar/velodyne:install",
#     ]
# )

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_library(
    name = "lidar_driver_component_lib",
    srcs = ["lidar_driver_component.cc"],
    hdrs = ["lidar_driver_component.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/drivers/lidar/common/driver_factory:lidar_driver_factory",
        "//modules/drivers/lidar/proto:config_cc_proto",
    ],
)

cc_binary(
    name = "liblidar_driver_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lidar_driver_component_lib"],
)

apollo_package()
cpplint()
