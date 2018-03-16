    
find_package(CUDA 8.0 REQUIRED)
include_directories(SYSTEM ${Cuda_ROOT}/include/)
link_directories(${Cuda_ROOT}/lib/)
link_directories(${Cuda_ROOT}/lib/stubs/)
