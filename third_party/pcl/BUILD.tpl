package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pcl_headers",
    hdrs = [
        ":pcl_include"
    ],
    strip_include_prefix = "pcl/include",
)

# TODO(storypku): split pcl into individual components
cc_library(
    name = "pcl",
    srcs = [":pcl_lib"],
    linkstatic = 1,
    deps = [
        ":pcl_headers",
        "@boost",
    ],
)

%{copy_rules}
