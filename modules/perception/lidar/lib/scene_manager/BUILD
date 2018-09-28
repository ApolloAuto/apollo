load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scene_manager",
    srcs = [
        "scene_manager.cc",
    ],
    hdrs = [
        "scene_manager.h",
    ],
    deps = [
        ":scene_service",
        "//modules/common/util",        
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/singleton",
        "//modules/perception/lib/thread",
        "//modules/perception/lidar/lib/scene_manager/proto:scene_manager_config_proto",
        "//modules/perception/proto:perception_config_schema_proto",
    ],
)

cc_library(
    name = "scene_service",
    hdrs = [
        "scene_service.h",
    ],
    deps = [
        "//modules/perception/lib/registerer",
    ],
)

cpplint()
