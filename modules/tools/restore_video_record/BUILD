load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])


filegroup(
    name = "readme",
    srcs = glob([
        "README.md",
    ]),
)

apollo_py_binary(
    name = "restore_video_record",
    srcs = ["restore_video_record.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/sensor_msgs:sensor_image_py_pb2",
    ],
)

# install(
#     name = "install",
#     data = [":readme"],
#     data_dest = "tools/restore_video_record",
#     py_dest = "tools/restore_video_record",
#     targets = [
#         ":restore_video_record",
#     ],
# )

apollo_package()