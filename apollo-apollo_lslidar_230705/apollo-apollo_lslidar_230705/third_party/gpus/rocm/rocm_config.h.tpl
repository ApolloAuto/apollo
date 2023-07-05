#ifndef ROCM_CONFIG_H_
#define ROCM_CONFIG_H_

#define TF_ROCM_AMDGPU_TARGETS "%{rocm_amdgpu_targets}"
#define TF_ROCM_TOOLKIT_PATH "%{rocm_toolkit_path}"
#define TF_ROCM_VERSION "%{rocm_version_number}"
#define TF_HIP_VERSION "%{hipruntime_version_number}"
#define TF_HIPBLAS_VERSION "%{hipblas_version_number}"
#define TF_ROCBLAS_VERSION "%{rocblas_version_number}"
#define TF_MIOPEN_VERSION "%{miopen_version_number}"
#define TF_MIGRAPHX_VERSION "%{migraphx_version_number}"

#endif  // ROCM_CONFIG_H_
