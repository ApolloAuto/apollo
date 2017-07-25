load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "backend",
    hdrs = [
        "dreamview.h",
    ],
    srcs = [
        "dreamview.cc",
        "main.cc",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common:apollo_app",
        "//modules/dreamview/backend/simulation_world:simulation_world_updater",
        "//modules/dreamview/backend/websocket",
        "@civetweb//:civetweb++",
    ],
)

cpplint()
