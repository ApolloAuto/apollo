"""Loads the FFTW3 library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
def clean_dep(dep):
    return str(Label(dep))

def repo():
    native.new_local_repository(
        name = "fftw3",
        build_file = clean_dep("//third_party/fftw3:fftw3.BUILD"),
        path = "/usr/include",
    )
