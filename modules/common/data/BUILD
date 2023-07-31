# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "vehicle_config_data",
    srcs = ["vehicle_param.pb.txt"],
)

filegroup(
    name = "global_flagfile",
    srcs = ["global_flagfile.txt"],
)

# install(
#     name = "install",
#     data_dest = "common/data",
#     data = [
#         ":global_flagfile",
#         ":vehicle_config_data",
#     ],
# )

apollo_package()