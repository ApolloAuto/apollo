package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lib_planning_common",
    srcs = [
        "planning_gflags.cc",
    ],
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:localization_proto",
        "@eigen//:eigen",
    ],
)
