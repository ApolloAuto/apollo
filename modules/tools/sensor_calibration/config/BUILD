# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "*.yaml",
        "*.config",
        "init_params/*"
    ]),
)

# install(
#     name = "install",
#     data = [":runtime_files"],
#     data_dest = "tools/sensor_calibration/config",
#     deps = ["//modules/tools/sensor_calibration/config/init_params:install"]
# )

apollo_package()