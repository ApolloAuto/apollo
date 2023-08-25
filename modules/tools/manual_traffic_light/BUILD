load("//tools:cpplint.bzl", "cpplint")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_component", "apollo_cc_binary", "apollo_cc_library")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "install",
#     library_dest = "tools/lib/manual_traffic_light",
#     data_dest = "tools/addition_data/manual_traffic_light",
#     data = [":conf_files"],
#     targets = [":libmanual_traffic_light_component.so"],
#     visibility = ["//visibility:public"],
# )

filegroup(
    name = "conf_files",
    srcs = [
        "manual_traffic_light.conf",
        "manual_traffic_light.dag",
        "manual_traffic_light.launch",
    ],
)

apollo_component(
    name = "libmanual_traffic_light_component.so",
    srcs = ["manual_traffic_light.cc"],
    copts = ["-DMODULE_NAME=\\\"manual_traffic_light\\\""],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/map:apollo_map",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
    ],
)

apollo_package()
cpplint()
