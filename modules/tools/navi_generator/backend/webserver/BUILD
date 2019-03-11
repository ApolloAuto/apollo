load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "navi_generator_websocket",
    srcs = [
        "navi_generator_websocket.cc",
    ],
    hdrs = [
        "navi_generator_websocket.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/time",
        "//modules/common/util:map_util",
        "//modules/common/util:string_util",
        "//third_party/json",
        "@civetweb//:civetweb++",
    ],
)

cpplint()
