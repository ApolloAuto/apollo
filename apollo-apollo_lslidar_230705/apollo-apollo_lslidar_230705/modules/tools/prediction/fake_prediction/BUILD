load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "tools/lib",
    targets = [":libfake_prediction_component.so"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libfake_prediction_component.so",
    srcs = ["fake_prediction.cc"],
    linkshared = True,
    linkstatic = False,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
    ],
)

cpplint()
