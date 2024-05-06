# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "install",
#     data_dest = "localization/addition_data/msf/params",
#     data = [
#         ":runtime_data",
#     ],
# )

filegroup(
    name = "runtime_data",
    srcs = glob([
        "gnss_params/*.yaml",
        "vehicle_params/*.yaml",
        "velodyne_params/*.yaml",
        "*.yaml",
    ]),
)

apollo_package()
