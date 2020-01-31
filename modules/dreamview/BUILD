load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "frontend",
    srcs = glob(["frontend/dist/**/*"]),
)

cc_binary(
    name = "dreamview",
    data = [
        ":frontend",
    ],
    deps = [
        "//modules/dreamview/backend",
        "//third_party:opencv",
    ],
)

cpplint()
