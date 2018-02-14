load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "navigation_dummy",
    srcs = ["navigation_dummy.cc"],
    deps = [
        "//modules/map/relative_map:relative_map_lib",
        "//modules/common",
        "@ros//:ros_common",
    ],
)

cpplint()
