load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "scene_manager_files",
    srcs = glob([
        "conf/**",
        "data/**",
    ])
)

cc_library(
    name = "scene_manager",
    srcs = ["scene_manager.cc"],
    hdrs = ["scene_manager.h"],
    alwayslink = True,
    deps = [
        ":scene_service",
        "//modules/common/util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/lib/thread",
        "//modules/perception/common/lidar/scene_manager/proto:scene_manager_config_cc_proto",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "//modules/perception/common:util",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
)

cc_library(
    name = "scene_service",
    hdrs = ["scene_service.h"],
    alwayslink = True,
    deps = [
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
)

apollo_package()
cpplint()
