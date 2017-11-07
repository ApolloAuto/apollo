load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_common_threadpool",
    hdrs = glob([
        "*.hpp", 
        "detail/*.hpp",
    ]),
    linkopts = [
        "-lboost_thread",
        "-lboost_system",
    ],
)

cpplint()