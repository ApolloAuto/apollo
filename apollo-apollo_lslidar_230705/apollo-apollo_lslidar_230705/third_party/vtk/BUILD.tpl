package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vtk_headers",
    hdrs = [
        ":vtk_include"
    ],
    # include_prefix = "third_party/vtk",
    strip_include_prefix = "vtk/include",
)

cc_library(
    name = "vtk",
    srcs = [":vtk_lib"],
    linkstatic = 1,
    deps = [
        ":vtk_headers",
    ],
)

%{copy_rules}
