load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_drivers_video_src",
#         ":install_drivers_video_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_drivers_video_src",
#     src_dir = ["."],
#     dest = "drivers-video/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_drivers_video_hdrs",
#     src_dir = ["."],
#     dest = "drivers-video/include",
#     filter = "*.h",
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "drivers-video/include",
#     data = [
#         "//modules/drivers/video/proto:video_h265cfg_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_drivers_video_py",
#     dest = "drivers-video/python/modules/drivers/video",
#     files = [
#         "//modules/drivers/video/proto:video_h265cfg_py_pb2"
#     ],
# )

filegroup(
    name = "package_data",
    srcs = [
        ":cyberfile.xml",
        ":drivers-video.BUILD",
    ],
)

# install(
#     name = "install_package_data",
#     data_dest = "drivers-video",
#     data = ["package_data"],
# )

# install(
#     name = "install",
#     data_dest = "drivers-video/addition_data",
#     library_dest = "drivers-video/lib",
#     data = [
#         ":runtime_files",
#     ],
#     targets = [
#         "libvideo_driver_component.so",
#     ],
#      deps = [
#         ":pb_hdrs", 
#         ":pb_drivers_video_py",
#         "install_package_data"
#     ]
# )

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ])
)

cc_binary(
    name = "libvideo_driver_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":video_driver_component_lib"],
)

cc_library(
    name = "video_driver_component_lib",
    srcs = ["video_driver_component.cc"],
    hdrs = ["video_driver_component.h"],
    copts = ['-DMODULE_NAME=\\"video\\"'],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/video:driver",
        "//modules/drivers/video:socket",
    ],
)

cc_library(
    name = "driver",
    srcs = ["driver.cc"],
    hdrs = [
        "driver.h",
        "socket_input.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/drivers/video/proto:video_h265cfg_cc_proto",
    ],
)

cc_library(
    name = "socket",
    srcs = ["socket_input.cc"],
    hdrs = [
        "input.h",
        "socket_input.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common/util",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
    ],
)

apollo_package()
cpplint()
