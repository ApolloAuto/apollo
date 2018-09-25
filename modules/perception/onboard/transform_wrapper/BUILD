load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transform_wrapper",
    srcs = [
        "transform_wrapper.cc",
    ],
    hdrs = [
        "transform_wrapper.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/perception/common/sensor_manager",
        "//modules/transform:tf2_buffer_lib",
        "@eigen",
    ],
)

cpplint()
