load("//tools/proto:proto.bzl", "apollo_py_binary")

# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "readme",
    srcs = [
        "README.md",
        "default.ini",
    ],
)

apollo_py_binary(
    name = "navi_config",
    srcs = ["navi_config.py"],
    data = ["default.ini"],
    deps = [
        "//modules/common_msgs/dreamview_msgs:hmi_config_py_pb2",
        "//modules/planning/planning_base/proto:planning_config_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

# install(
#     name = "install",
#     data = [":readme"],
#     data_dest = "tools/navigation/config",
#     py_dest = "tools/navigation/config",
#     targets = [
#         "navi_config",
#     ]
# )

apollo_package()
