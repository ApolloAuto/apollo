package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lib_reference_line",
    srcs = [
        "reference_point.cc",
        "reference_line.cc",
    ],
    hdrs = [
        "reference_point.h",
        "reference_line.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:path_point_proto",
        "//modules/map/pnc_map:pnc_path",
        "//modules/common/math:math",
        "//modules/planning/math:double",
        "//modules/planning/math:interpolation",
    ],
)
