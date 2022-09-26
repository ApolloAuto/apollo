load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "calibration",
    data = [
        ":calibrated_vehicles",
        ":cyberfile.xml",
        ":calibration.BUILD",
    ],
)
install_src_files(
    name = "install_src",
    deps = [
        ":install_calibration_src",
        ":install_calibration_hdrs"
    ],
)

install_src_files(
    name = "install_calibration_src",
    src_dir = ["."],
    dest = "calibration/src",
    filter = "*",
)

install_src_files(
    name = "install_calibration_hdrs",
    src_dir = ["."],
    dest = "calibration/include",
    filter = "*.h",
)

filegroup(
    name = "calibrated_vehicles",
    srcs = glob([
        "data/**",
    ]),
)
