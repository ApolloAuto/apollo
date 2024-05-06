load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "plot_control",
    srcs = ["plot_control.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/control_msgs:control_cmd_py_pb2",
    ],
)

apollo_py_binary(
    name = "realtime_test",
    srcs = ["realtime_test.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)

# install(
#     name = "install",
#     py_dest = "tools/plot_control",
#     targets = [
#         ":plot_control",
#         ":realtime_test"
#     ]
# )

apollo_package()