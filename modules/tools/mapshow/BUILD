load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "readme",
    srcs = [
        "README.md",
    ],
)

apollo_py_binary(
    name = "mapshow",
    srcs = ["mapshow.py"],
    deps = [
        "//modules/tools/mapshow/libs:localization",
        "//modules/tools/mapshow/libs:map",
        "//modules/tools/mapshow/libs:path",
    ],
)

apollo_py_binary(
    name = "roadshow",
    srcs = ["roadshow.py"],
    deps = [
        "//modules/tools/mapshow/libs:map",
    ],
)

# install(
#     name = "install",
#     data = [":readme"],
#     data_dest = "tools/mapshow",
#     py_dest = "tools/mapshow",
#     targets = [
#         "mapshow",
#         "roadshow",
#     ]
# )

apollo_package()