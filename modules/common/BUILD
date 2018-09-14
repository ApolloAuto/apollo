load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    deps = [
        ":log",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/util",
        "//framework:cybertron",
    ],
)

cc_library(
    name = "macro",
    hdrs = [
        "macro.h",
    ],
    deps = [
        "//framework:cybertron",
    ],
)

cc_library(
    name = "log",
    hdrs = [
        "log.h",
    ],
    deps = [
        "//framework:cybertron",
    ],
)

# cc_library(
#     name = "apollo_app",
#     srcs = [
#         "apollo_app.cc",
#     ],
#     hdrs = [
#         "apollo_app.h",
#     ],
#     deps = [
#         ":log",
#         "//framework:cybertron",
#         "//modules/common/status",
#         "//modules/common/util:string_util",
#     ],
# )

cpplint()
