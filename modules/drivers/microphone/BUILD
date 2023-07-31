load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MICROPHONE_COPTS = ['-DMODULE_NAME=\\"microphone\\"']

cc_binary(
    name = "libmicrophone_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":microphone_component_lib",
    ],
)

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_drivers_microphone_src",
#         ":install_drivers_microphone_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_drivers_microphone_src",
#     src_dir = ["."],
#     dest = "drivers-microphone/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_drivers_microphone_hdrs",
#     src_dir = ["."],
#     dest = "drivers-microphone/include",
#     filter = "*.h",
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "drivers-microphone/include",
#     data = [
#         "//modules/drivers/microphone/proto:audio_cc_proto",
#         "//modules/drivers/microphone/proto:microphone_config_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_drivers_microphone_py",
#     dest = "drivers-microphone/python/modules/drivers/microphone",
#     files = [
#         "//modules/drivers/microphone/proto:microphone_config_py_pb2",
#         "//modules/drivers/microphone/proto:audio_py_pb2",
#     ],
# )

filegroup(
    name = "package_data",
    srcs = [
        ":cyberfile.xml",
        ":drivers-microphone.BUILD",
    ],
)

# install(
#     name = "install_package_data",
#     data_dest = "drivers-microphone",
#     data = ["package_data"],
# )

# install(
#     name = "install",
#     data_dest = "drivers-microphone/addition_data",
#     library_dest = "drivers-microphone/lib",
#     data = [
#         ":runtime_data",
#     ],
#     targets = [
#         ":libmicrophone_component.so",
#     ],
#     deps = [
#         ":pb_hdrs",
#         ":pb_drivers_microphone_py",
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

cc_library(
    name = "microphone_component_lib",
    srcs = ["microphone_component.cc"],
    hdrs = ["microphone_component.h"],
    copts = MICROPHONE_COPTS,
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/microphone:respeaker",
        "//modules/drivers/microphone/proto:audio_cc_proto",
        "//modules/drivers/microphone/proto:microphone_config_cc_proto",
    ],
)

cc_library(
    name = "respeaker",
    srcs = ["respeaker.cc"],
    hdrs = ["respeaker.h"],
    copts = MICROPHONE_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/microphone/proto:microphone_config_cc_proto",
        "@portaudio",
    ],
)

apollo_package()
cpplint()
