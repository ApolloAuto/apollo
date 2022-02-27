def clean_dep(dep):
    return str(Label(dep))
def repo():
     native.new_local_repository(
        name = "rocm",
        build_file = clean_dep("//third_party/gpus/rocm:rocm.BUILD"),
        path = "/opt/rocm",
    )