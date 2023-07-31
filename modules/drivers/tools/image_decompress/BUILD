load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_drivers_src",
#         ":install_drivers_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_drivers_src",
#     src_dir = ["."],
#     dest = "drivers-image-decompress/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_drivers_hdrs",
#     src_dir = ["."],
#     dest = "drivers-image-decompress/include",
#     filter = "*.h",
# )

# install(
#     name = "pb_hdrs",
#     data_dest = "drivers-image-decompress/include",
#     data = [
#         "//modules/drivers/tools/image_decompress/proto:config_cc_proto",
#     ],
# )

# install_files(
#     name = "pb_drivers_py",
#     dest = "drivers-image-decompress/python/modules/drivers/tools/image_decompress",
#     files = [
#         "//modules/drivers/tools/image_decompress/proto:config_py_pb2",
#     ],
# )

filegroup(
    name = "package_data",
    srcs = [
        ":cyberfile.xml",
        ":drivers-image-decompress.BUILD",
    ],
)

# install(
#     name = "install_package_data",
#     data_dest = "drivers-image-decompress",
#     data = ["package_data"],
# )

# install(
#     name = "install",
#     library_dest = "drivers-image-decompress/lib",
#     data_dest = "drivers-image-decompress/addition_data",
#     data = [
#         ":runtime_data",
#     ],
#     targets = [
#         ":libimage_decompress.so",
#     ],
#     deps = [
#         ":pb_hdrs", 
#         ":pb_drivers_py",
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

cc_library(
    name = "image_decompress_lib",
    srcs = ["image_decompress.cc"],
    hdrs = ["image_decompress.h"],
    copts = ['-DMODULE_NAME=\\"image_decompress\\"'],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/drivers/tools/image_decompress/proto:config_cc_proto",
        "@opencv//:imgcodecs",
    ],
)

cc_test(
    name = "image_decompress_test",
    size = "small",
    srcs = ["image_decompress_test.cc"],
    deps = [
        ":image_decompress_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "libimage_decompress.so",
    linkshared = True,
    linkstatic = True,
    deps = [":image_decompress_lib"],
)

apollo_package()
cpplint()
