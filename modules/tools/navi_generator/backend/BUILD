load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "backend",
    srcs = [
        "main.cc",
        "navi_generator.cc",
    ],
    hdrs = [
        "navi_generator.h",
    ],
    deps = [
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util:util",
        "//modules/tools/navi_generator/backend/hmi:topics_updater",
        "//modules/tools/navi_generator/backend/webserver:navi_generator_websocket",
        "@civetweb//:civetweb++",
    ],
)

cpplint()
