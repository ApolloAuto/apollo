load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_drivers_canbus_src",
#         ":install_drivers_canbus_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_drivers_canbus_src",
#     src_dir = ["."],
#     dest = "drivers-canbus/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_drivers_canbus_hdrs",
#     src_dir = ["."],
#     dest = "drivers-canbus/include",
#     filter = "*.h",
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "drivers-canbus/include",
#     data = [
#         "//modules/drivers/canbus/proto:sensor_canbus_conf_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_drivers_canbus_py",
#     dest = "drivers-canbus/python/modules/drivers/canbus",
#     files = [
#         "//modules/drivers/canbus/proto:sensor_canbus_conf_py_pb2",
#     ],
# )

# install(
#     name = "install",
#     library_dest = "drivers-canbus/lib",
#     targets = [
#         ":libsensor_gflags.so",
#         ":libsensor_canbus.so"
#     ],
#     data = [
#         ":cyberfile.xml",
#         "drivers-canbus.BUILD",
#     ],
#     data_dest = "drivers-canbus",
#     visibility = ["//visibility:public"],
#     deps = [
#         ":pb_drivers_canbus_py",
#         ":pb_hdrs", 
#     ]
# )

cc_binary(
    name = "libsensor_gflags.so",
    srcs = ["sensor_gflags.cc", "sensor_gflags.h"],
    linkshared = True,
    linkstatic = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "sensor_gflags",
    srcs = ["libsensor_gflags.so"],
    hdrs = ["sensor_gflags.h"],
    alwayslink = True,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "sensor_canbus_lib",
    srcs = ["libsensor_canbus.so"],
    alwayslink = True,
    deps = [
        ":sensor_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)

cc_binary(
    name = "libsensor_canbus.so",
    srcs = ["sensor_canbus.h"],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":sensor_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)

apollo_package()
cpplint()
