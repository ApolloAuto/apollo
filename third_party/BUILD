package(default_visibility = ["//visibility:public"])

# Libs provided by the system.

# libadolc-dev
cc_library(
    name = "adolc",
    linkopts = ["-ladolc"],
)

# libboost-dev
# TODO(xiaoxq): This is not a recommended way to introduce boost. Instead, we'd
# better leverage things like https://github.com/nelhage/rules_boost
# However, another library, libpcl-dev depends on libboost-dev. So we decide to
# reuse it instead of making duplication.
cc_library(
    name = "boost",
    linkopts = [
        "-lboost_system",
        "-lboost_filesystem",
        "-lboost_program_options",
        "-lboost_thread",
        "-lboost_signals",
    ],
)

# libcaffe-cuda-dev
cc_library(
    name = "caffe",
    linkopts = [
        "-lcaffe",
        "-lcublas",
    ],
)

cc_library(
    name = "cuda",
    includes = ["include"],
    linkopts = [
        "-lgomp",
        "-L/usr/local/cuda/lib64",
        "-lOpenCL",
        "-lcublas",
        "-lcudart",
        "-lcudnn",
        "-lcufft",
        "-lcufftw",
        "-lcuinj64",
        "-lcurand",
        "-lcusolver",
        "-lcusparse",
        "-lnppc",
        "-lnppial",
        "-lnppicc",
        "-lnppicom",
        "-lnppidei",
        "-lnppif",
        "-lnppig",
        "-lnppim",
        "-lnppist",
        "-lnppisu",
        "-lnppitc",
        "-lnpps",
        "-lnvToolsExt",
        "-lnvblas",
        "-lnvgraph",
        "-lnvrtc-builtins",
        "-lnvrtc",
        "-L/usr/local/cuda/lib64/stubs",
        "-lcublas",
        "-lcuda",
        "-lcufft",
        "-lcufftw",
        "-lcurand",
        "-lcusolver",
        "-lcusparse",
        "-lnppc",
        "-lnppial",
        "-lnppicc",
        "-lnppicom",
        "-lnppidei",
        "-lnppif",
        "-lnppig",
        "-lnppim",
        "-lnppist",
        "-lnppisu",
        "-lnppitc",
        "-lnpps",
        "-lnvgraph",
        "-lnvidia-ml",
        "-lnvrtc",
    ],
)

# install_fastrtps.sh
cc_library(
    name = "fastcdr",
    linkopts = [
        "-L/usr/local/lib",
        "-lfastcdr",
    ],
)

# install_fastrtps.sh
cc_library(
    name = "fastrtps",
    linkopts = [
        "-L/usr/local/lib",
        "-lfastrtps",
    ],
    deps = [
        ":fastcdr",
    ],
)

# libavcodec-dev
# libavutil-dev
# libswresample-dev
cc_library(
    name = "ffmpeg",
    linkopts = [
        "-lavcodec",
        "-lavutil",
        "-lswresample",
    ],
)

cc_library(
    name = "libtorch",
    deps = select({
        "//tools/platforms:use_gpu": [
            "@pytorch_gpu",
        ],
        "//conditions:default": [
            "@pytorch",
        ],
    }),
)

# coinor-libipopt-dev
cc_library(
    name = "ipopt",
    linkopts = ["-lipopt"],
)

# libncurses5-dev
cc_library(
    name = "ncurses",
    linkopts = ["-lncurses"],
)

# libomp-dev
cc_library(
    name = "openmp",
    linkopts = ["-lomp"],
)

# libopencv-dev
cc_library(
    name = "opencv",
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
        "-lopencv_imgcodecs",
    ],
)

# libpoco-dev
cc_library(
    name = "PocoFoundation",
    linkopts = ["-lPocoFoundation"],
)

# libpcl-dev
cc_library(
    name = "pcl",
    linkopts = [
        "-lpcl_common",
        "-lpcl_features",
        "-lpcl_filters",
        "-lpcl_io_ply",
        "-lpcl_io",
        "-lpcl_kdtree",
        "-lpcl_keypoints",
        "-lpcl_octree",
        "-lpcl_outofcore",
        "-lpcl_people",
        "-lpcl_recognition",
        "-lpcl_registration",
        "-lpcl_sample_consensus",
        "-lpcl_search",
        "-lpcl_segmentation",
        "-lpcl_surface",
        "-lpcl_tracking",
        "-lpcl_visualization",
    ],
)

cc_library(
    name = "pthread",
    linkopts = ["-pthread"],
)

# python3-dev
cc_library(
    name = "python3",
    linkopts = ["-lpython3.6m"],
)

cc_library(
    name = "rt",
    linkopts = ["-lrt"],
)

# libsqlite3-dev
cc_library(
    name = "sqlite3",
    linkopts = ["-lsqlite3"],
)

# uuid-dev
cc_library(
    name = "uuid",
    linkopts = ["-luuid"],
)
