load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "esdcan_test_app",
    srcs = ["esdcan_test_app.cc"],
    deps = [
        "//external:gflags",
        "//modules/monitor/hardware/can/esdcan:esdcan_checker_lib",
    ],
)

cpplint()
