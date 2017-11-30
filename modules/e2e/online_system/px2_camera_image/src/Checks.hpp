#ifndef SAMPLES_COMMON_CHECKS_HPP__
#define SAMPLES_COMMON_CHECKS_HPP__

#include <dw/gl/GL.h>

#include <cuda_runtime.h>

#include <iostream>
#include <string>
#include <stdexcept>

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// print GL error with location
inline void getGLError(int line, const char *file, const char *function)
{
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        std::cerr << file << " in function " << function << " in line " << line << ": glError: 0x"
                  << std::hex << error << std::dec << std::endl;
        exit(1);
    }
}

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCTION__
#endif

// macro to easily check for GL errors
#define CHECK_GL_ERROR() getGLError(__LINE__, __FILE__, __PRETTY_FUNCTION__);

//------------------------------------------------------------------------------
// macro to easily check for dw errors
#define CHECK_DW_ERROR(x) { \
                    dwStatus result = x; \
                    if(result!=DW_SUCCESS) \
                        throw std::runtime_error(std::string("DW Error ") \
                                                + dwGetStatusName(result) \
                                                + std::string(" executing DW function:\n " #x) \
                                                + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
                    };

//------------------------------------------------------------------------------
// macro to easily check for cuda errors
#define CHECK_CUDA_ERROR(x) { \
                    x; \
                    auto result = cudaGetLastError(); \
                    if(result != cudaSuccess) \
                        throw std::runtime_error(std::string("CUDA Error ") \
                                                + cudaGetErrorString(result) \
                                                + std::string(" executing CUDA function:\n " #x) \
                                                + std::string("\n at " __FILE__ ":") + std::to_string(__LINE__)); \
                    };

#endif // SAMPLES_COMMON_CHECKS_HPP__
