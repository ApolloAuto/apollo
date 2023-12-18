load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "libfake_prediction_component.so",
    srcs = ["fake_prediction.cc"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
    ],
)

apollo_package()
cpplint()
