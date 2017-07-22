package(default_visibility = ["//visibility:public"])

cc_library(
    name = "st_point",
    srcs = [
        "st_point.cc",
    ],
    hdrs = [
        "st_point.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common/math",
        "//modules/planning/math:double",
    ],
)

cc_library(
    name = "speed_data",
    srcs = [
        "speed_data.cc",
    ],
    hdrs = [
        "speed_data.h",
    ],
    deps = [
        ":st_point",
        "//external:gflags",
        "//modules/common/math",
        "//modules/common/proto:path_point_proto",
        "//modules/planning/math:double",
        "//modules/planning/proto:planning_proto",
    ],
)
