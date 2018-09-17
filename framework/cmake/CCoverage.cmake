#Find bullseye executables first

set(BULLSEYE_CC "${COV_HOME}/bin/g++")
set(BULLSEYE_COV_ENABLE "${COV_HOME}/bin/cov01")

#This will go ahead and find the programs, if it cant find, it will report error
#find_package_handle_standard_args(BULLSEYE DEFAULT_MSG BULLSEYE_CC BULLSEYE_COV_ENABLE)

#Set the CMake compiler to bullseye compiler.
#This is important as Bullseye (behind the scenes) will invoke the original preferred compiler after its own book-keeping.
#Enable coverage
if (DEFINED CCover)
    execute_process(COMMAND ${BULLSEYE_COV_ENABLE} -1 RESULT_VARIABLE RES)
    if (${RES})
        message(FATAL_ERROR "Could not enable CCoverage.")
    endif()
    SET(CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER_CCOVER} CACHE STRING "" FORCE)
    message("Turn On CCoverage, set CXX_Compiler:" ${CMAKE_CXX_COMPILER_CCOVER})
else (DEFINED CCover)
    execute_process(COMMAND ${BULLSEYE_COV_ENABLE} -0 RESULT_VARIABLE RES)
    if (${RES})
        message(FATAL_ERROR "Could not disable CCoverage.")
    endif()
    message("Turn Off CCoverage, set CXX_Compiler:" ${CMAKE_CXX_COMPILER})
endif()
