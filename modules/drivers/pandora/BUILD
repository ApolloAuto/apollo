load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

genrule(
    name = "pandora_genrule",
    srcs = glob([
        "**/*.cc",
        "**/*.h",
        "build_and_install.sh",
    ]),
    outs = ["pandora_genrule.log"],
    cmd = "bash modules/drivers/pandora/build_and_install.sh > $@",
)

filegroup(
    name = "genrule_success",
    srcs = ["pandora_genrule.log"],
)

cpplint()
