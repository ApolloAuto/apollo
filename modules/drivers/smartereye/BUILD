load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CAMERA_COPTS = ['-DMODULE_NAME=\\"camera\\"']

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_drivers_smartereye_src",
#         ":install_drivers_smartereye_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_drivers_smartereye_src",
#     src_dir = ["."],
#     dest = "drivers-smartereye/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_drivers_smartereye_hdrs",
#     src_dir = ["."],
#     dest = "drivers-smartereye/include",
#     filter = "*.h",
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "drivers-smartereye/include",
#     data = [
#         "//modules/drivers/smartereye/proto:config_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_drivers_smartereye_py",
#     dest = "drivers-smartereye/python/modules/drivers/smartereye",
#     files = [
#         "//modules/drivers/smartereye/proto:config_py_pb2",
#     ],
# )

filegroup(
    name = "package_data",
    srcs = [
        ":cyberfile.xml",
        ":drivers-smartereye.BUILD",
    ],
)

# install(
#     name = "install_package_data",
#     data_dest = "drivers-smartereye",
#     data = ["package_data"],
# )

# install(
#     name = "install",
#     data_dest = "drivers-smartereye/addition_data",
#     library_dest = "drivers-smartereye/lib",
#     data = [
#         ":runtime_data",
#     ],
#     targets = [
#         ":libsmartereye_component.so",
#     ],
#     deps = [
#         ":pb_hdrs", 
#         ":pb_drivers_smartereye_py",
#         "install_package_data"
#     ]
# )

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_binary(
    name = "libsmartereye_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":compress_component_lib",
        ":smartereye_component_lib",
    ],
)

cc_library(
    name = "smartereye_component_lib",
    srcs = ["smartereye_component.cc"],
    hdrs = ["smartereye_component.h"],
    alwayslink = True,
    copts = CAMERA_COPTS,
    deps = [
        ":smartereye_device",
        ":smartereye_handler",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/sensor_msgs:smartereye_cc_proto",
        "//third_party/camera_library/smartereye",
    ],
)

cc_library(
    name = "smartereye_device",
    srcs = ["smartereye_device.cc"],
    hdrs = ["smartereye_device.h"],
    copts = CAMERA_COPTS,
    deps = [
        ":smartereye_handler",
        "//cyber",
        "//modules/drivers/smartereye/proto:config_cc_proto",
        "//third_party/camera_library/smartereye",
    ],
)

cc_library(
    name = "smartereye_handler",
    srcs = ["smartereye_handler.cc"],
    hdrs = ["smartereye_handler.h"],
    copts = CAMERA_COPTS,
    deps = [
        "//cyber",
        "//third_party/camera_library/smartereye",
    ],
)

cc_library(
    name = "compress_component_lib",
    srcs = ["compress_component.cc"],
    hdrs = ["compress_component.h"],
    copts = CAMERA_COPTS,
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/sensor_msgs:smartereye_cc_proto",
        "//modules/drivers/smartereye/proto:config_cc_proto",
        "@opencv//:highgui",
    ],
)

apollo_package()
cpplint()
