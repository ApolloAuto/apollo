package(default_visibility = ["//visibility:public"])

cc_library(
    name = "speed",
    srcs = [
        "st_point.cc",
        "speed_point.cc",
        "speed_data.cc",
    ],
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//external:gflags",
        "//modules/common/math",
        "//modules/planning/math:double",
    ],
)
