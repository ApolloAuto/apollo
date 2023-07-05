load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "common/lib",
    data_dest = "common",
    targets = [
        ":libconfig_gflags.so",
        ":libvehicle_config_helper.so",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libconfig_gflags.so",
    srcs = [
        "config_gflags.cc",
        "config_gflags.h",
    ],
    linkshared = True,
    linkstatic = True,
    data = [
        "//modules/common/data:global_flagfile",
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "config_gflags",
    srcs = ["libconfig_gflags.so"],
    hdrs = ["config_gflags.h"],
    data = [
        "//modules/common/data:global_flagfile",
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libvehicle_config_helper.so",
    srcs = [
        "vehicle_config_helper.cc",
        "vehicle_config_helper.h",
    ],
    linkshared = True,
    linkstatic = True,
    data = [
        "//modules/common/data:vehicle_config_data",
    ],
    deps = [
        ":config_gflags",
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common/math",
    ],
)

cc_library(
    name = "vehicle_config_helper",
    srcs = ["libvehicle_config_helper.so"],
    hdrs = ["vehicle_config_helper.h"],
    data = [
        "//modules/common/data:vehicle_config_data",
    ],
    deps = [
        ":config_gflags",
        "//cyber",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "vehicle_config_helper_test",
    size = "small",
    srcs = ["vehicle_config_helper_test.cc"],
    deps = [
        ":vehicle_config_helper",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
