# Apollo external dependencies that can be loaded in WORKSPACE files.
load("//third_party/gpus:cuda_configure.bzl", "cuda_configure")
load("//third_party/tensorrt:tensorrt_configure.bzl", "tensorrt_configure")
load("//third_party/py:python_configure.bzl", "python_configure")
load("//third_party/vtk:vtk_configure.bzl", "vtk_configure")
load("//third_party/adolc:workspace.bzl", adolc = "repo")
load("//third_party/adv_plat:workspace.bzl", adv_plat = "repo")
load("//third_party/ad_rss_lib:workspace.bzl", ad_rss_lib = "repo")
load("//third_party/boost:workspace.bzl", boost = "repo")
load("//third_party/civetweb:workspace.bzl", civetweb = "repo")
load("//third_party/cpplint:workspace.bzl", cpplint = "repo")
load("//third_party/eigen3:workspace.bzl", eigen = "repo")
load("//third_party/fastrtps:workspace.bzl", fastrtps = "repo")
load("//third_party/ffmpeg:workspace.bzl", ffmpeg = "repo")
load("//third_party/gflags:workspace.bzl", gflags = "repo")

# load("//third_party/glew:workspace.bzl", glew = "repo")
load("//third_party/glog:workspace.bzl", glog = "repo")
load("//third_party/gtest:workspace.bzl", gtest = "repo")
load("//third_party/ipopt:workspace.bzl", ipopt = "repo")
load("//third_party/local_integ:workspace.bzl", local_integ = "repo")
load("//third_party/libtorch_gpu:workspace.bzl", libtorch_gpu = "repo")
load("//third_party/libtorch_cpu:workspace.bzl", libtorch_cpu = "repo")
load("//third_party/lz4:workspace.bzl", lz4 = "repo")
load("//third_party/npp:workspace.bzl", npp = "repo")
load("//third_party/opencv:workspace.bzl", opencv = "repo")
load("//third_party/osqp:workspace.bzl", osqp = "repo")
load("//third_party/pcl:workspace.bzl", pcl = "repo")
load("//third_party/poco:workspace.bzl", poco = "repo")
load("//third_party/proj4:workspace.bzl", proj4 = "repo")
load("//third_party/protobuf:workspace.bzl", protobuf = "repo")
load("//third_party/qpOASES:workspace.bzl", qpOASES = "repo")
load("//third_party/qt5:workspace.bzl", qt5 = "repo")
load("//third_party/tf2:workspace.bzl", tf2 = "repo")
load("//third_party/tinyxml2:workspace.bzl", tinyxml2 = "repo")
load("//third_party/yaml_cpp:workspace.bzl", yaml_cpp = "repo")

def initialize_third_party():
    """ Load third party repositories.  See above load() statements. """

    adolc()
    adv_plat()
    ad_rss_lib()
    boost()
    cpplint()
    civetweb()
    eigen()
    fastrtps()
    ffmpeg()
    gflags()
    glog()
    gtest()
    ipopt()
    local_integ()
    libtorch_cpu()
    libtorch_gpu()
    lz4()
    npp()
    opencv()
    osqp()
    pcl()
    poco()
    proj4()
    protobuf()
    qpOASES()
    qt5()
    tf2()
    tinyxml2()
    yaml_cpp()

# Define all external repositories required by
def apollo_repositories():
    ##=============== bazel_skylib ===============##
    http_archive(
        name = "bazel_skylib",
        sha256 = "1dde365491125a3db70731e25658dfdd3bc5dbdfd11b840b3e987ecf043c7ca0",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/0.9.0/bazel_skylib-0.9.0.tar.gz"],
    )

    load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

    bazel_skylib_workspace()

    ##=============== rules_proto ===============##
    http_archive(
        name = "rules_proto",
        sha256 = "602e7161d9195e50246177e7c55b2f39950a9cf7366f74ed5f22fd45750cd208",
        strip_prefix = "rules_proto-97d8af4dc474595af3900dd85cb3a29ad28cc313",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/rules_proto/archive/97d8af4dc474595af3900dd85cb3a29ad28cc313.tar.gz",
            "https://github.com/bazelbuild/rules_proto/archive/97d8af4dc474595af3900dd85cb3a29ad28cc313.tar.gz",
        ],
    )

    load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")

    rules_proto_dependencies()
    rules_proto_toolchains()

    ##================== grpc =====================##
	http_archive(
    	name = "com_github_grpc_grpc",
    	sha256 = "419dba362eaf8f1d36849ceee17c3e2ff8ff12ac666b42d3ff02a164ebe090e9",
    	strip_prefix = "grpc-1.30.0",
    	urls = ["https://github.com/grpc/grpc/archive/v1.30.0.tar.gz"],
    )

    load("@com_github_grpc_grpc//bazel:grpc_deps.bzl", "grpc_deps")
    grpc_deps()

	load("@com_github_grpc_grpc//bazel:grpc_extra_deps.bzl", "grpc_extra_deps")
	grpc_extra_deps()

    ##=============== local_config_xxx ==================##
    cuda_configure(name = "local_config_cuda")
    tensorrt_configure(name = "local_config_tensorrt")
    python_configure(name = "local_config_python")
    vtk_configure(name = "local_config_vtk")

    initialize_third_party()
