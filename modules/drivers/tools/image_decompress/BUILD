load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_component", "apollo_cc_library", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_cc_test(
    name = "image_decompress_test",
    size = "small",
    srcs = ["image_decompress_test.cc"],
    deps = [
        ":DO_NOT_IMPORT_image_decompress",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_component(
    name = "libimage_decompress.so",
    srcs = ["image_decompress.cc"],
    hdrs = ["image_decompress.h"],
    copts = ['-DMODULE_NAME=\\"image_decompress\\"'],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/drivers/tools/image_decompress/proto:config_cc_proto",
        "@opencv//:imgcodecs",
    ],
)

apollo_package()
cpplint()
