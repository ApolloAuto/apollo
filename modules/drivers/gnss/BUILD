load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

GNSS_COPTS = ['-DMODULE_NAME=\\"gnss\\"']

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_drivers_gnss_src",
#         ":install_drivers_gnss_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_drivers_gnss_src",
#     src_dir = ["."],
#     dest = "drivers-gnss/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_drivers_gnss_hdrs",
#     src_dir = ["."],
#     dest = "drivers-gnss/include",
#     filter = "*.h",
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "drivers-gnss/include",
#     data = [
#         "//modules/drivers/gnss/proto:config_cc_proto",
#         "//modules/drivers/gnss/proto:gnss_status_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_drivers_gnss_py",
#     dest = "drivers-gnss/python/modules/drivers/gnss",
#     files = [
#         "//modules/drivers/gnss/proto:config_py_pb2",
#         "//modules/drivers/gnss/proto:gnss_status_py_pb2",
#     ],
# )

cc_binary(
    name = "libgnss_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":gnss_component_lib"],
)

cc_library(
    name = "gnss_component_lib",
    srcs = ["gnss_component.cc"],
    hdrs = ["gnss_component.h"],
    copts = GNSS_COPTS,
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common/monitor_log",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/drivers/gnss/stream:gnss_stream",
    ],
)

filegroup(
    name = "package_data",
    srcs = [
        ":cyberfile.xml",
        ":drivers-gnss.BUILD",
    ],
)

# install(
#     name = "install_package_data",
#     data_dest = "drivers-gnss",
#     data = ["package_data"],
# )

# install(
#     name = "install",
#     data_dest = "drivers-gnss/addition_data",
#     library_dest = "drivers-gnss/lib",
#     data = [
#         ":runtime_data",
#     ],
#     targets = [
#         ":libgnss_component.so",
#     ],
#     deps = [
#         ":pb_hdrs",
#         ":pb_drivers_gnss_py",
#         ":install_package_data",
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

apollo_package()
cpplint()
