load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
# load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "3rd-camera-library",
    data = [
        ":cyberfile.xml",
        ":3rd-camera-library.BUILD"
    ],
)

install_src_files(
    name = "lib_placeholder",
    src_dir = ["."],
    dest = "3rd-camera-library/lib",
    filter = "3rd-camera-library.BUILD"
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "3rd-camera-library/src",
    filter = "*",
    deps = ["//third_party/camera_library/smartereye:headers", ":lib_placeholder"]
)
# apollo_package()
