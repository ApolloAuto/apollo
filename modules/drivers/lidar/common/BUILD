load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_test")

apollo_cc_library(
    name = "lidar_common",
    copts = [],
    hdrs = [
        "sync_buffering.h",
        "lidar_component_base_impl.h",
        "lidar_component_base.h",
        "util.h",
    ],
    srcs = [
        "util.cpp",
    ],
    deps = [
      "@boost",
      "//cyber",
      "//modules/drivers/lidar/common/proto:lidar_config_base_proto",
    ],
)

apollo_package()

cpplint()
