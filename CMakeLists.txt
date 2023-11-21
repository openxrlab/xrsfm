cmake_minimum_required(VERSION 3.16)

project(xrsfm)

option(XRPRIMER_ENABLED "Buinding based on XRPRIMER, if available" ON)
option(CUDA_ENABLED "Whether to enable CUDA, if available" ON)
set(CUDA_ARCHS "Auto" CACHE STRING "List of CUDA architectures for which to \
generate code, e.g., Auto, All, Maxwell, Pascal, ...")


################################################################################
# Find packages
################################################################################

if(XRPRIMER_ENABLED)
    set(XRPrimer_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../xrprimer/install/lib/cmake")
    find_package(XRPrimer REQUIRED)
else ()
    find_package(Ceres REQUIRED)
    find_package(OpenCV REQUIRED)
    if(Ceres_VERSION VERSION_GREATER 2.1)
        message(FATAL_ERROR "Ceres version ${Ceres_VERSION} is not supported. Please install a version less than 2.2.")
    endif()
endif ()

set(OpenGL_GL_PREFERENCE "LEGACY")#for const
find_package(OpenGL REQUIRED)

find_package(GLEW REQUIRED)

if (CUDA_ENABLED)
    find_package(CUDA ${CUDA_MIN_VERSION} QUIET)
    message(STATUS "found")
endif ()

################################################################################
# Compiler specific configuration
################################################################################

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++14 -msse4 -fopenmp -fpermissive -w -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -g")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g -ggdb")
set(CMAKE_BUILD_TYPE "Release")

if (OPENGL_ENABLED)
    add_definitions("-DOPENGL_ENABLED")
    message(STATUS "Enabling OpenGL support")
else ()
    message(STATUS "Disabling OpenGL support")
endif ()

if (CUDA_FOUND)
    if (CUDA_ENABLED)
        add_definitions("-DCUDA_ENABLED")

        include(${CMAKE_CURRENT_SOURCE_DIR}/SelectCudaComputeArch.cmake)

        CUDA_SELECT_NVCC_ARCH_FLAGS(CUDA_ARCH_FLAGS ${CUDA_ARCHS})

        set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${CUDA_ARCH_FLAGS}")
        set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -D_FORCE_INLINES")
        set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Wno-deprecated-gpu-targets")
        set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --use_fast_math")
        set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --default-stream per-thread")

        add_definitions("-D_MWAITXINTRIN_H_INCLUDED")

        message(STATUS "Enabling CUDA support (version: ${CUDA_VERSION_STRING},"
                " archs: ${CUDA_ARCH_FLAGS_readable})")
    else ()
        set(CUDA_FOUND OFF)
        message(STATUS "Disabling CUDA support")
    endif ()
else ()
    set(CUDA_ENABLED OFF)
    if (CUDA_VERSION_STRING)
        message(STATUS "Disabling CUDA support (found version "
                "${CUDA_VERSION_STRING} but >= ${CUDA_MIN_VERSION} required)")
    else ()
        message(STATUS "Disabling CUDA support!")
    endif ()
endif ()

################################################################################
# Add sources
################################################################################

set(LIB_INSTALL_DIR ${PROJECT_SOURCE_DIR}/lib)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(3rdparty/SiftGPU)
add_subdirectory(3rdparty/apriltag)

add_library(xrsfm
        src/base/map.cc

        src/feature/sift_extractor.cc
        src/feature/feature_extraction.cc
        src/feature/feature_processing.cc
        src/feature/match_expansion.cc

        src/geometry/map_initializer.cc
        src/geometry/error_detector.cc
        src/geometry/error_corrector.cc

        src/geometry/pnp.cc
        src/geometry/essential.cc
        src/geometry/triangluate_svd.cc
        src/geometry/track_processor.cc

        src/geometry/colmap/base/triangulation.cc
        src/geometry/colmap/estimators/absolute_pose.cc
        src/geometry/colmap/estimators/triangulation.cc
        src/geometry/colmap/estimators/polynomial.cc
        src/geometry/colmap/estimators/fundamental_matrix.cc
        src/geometry/colmap/estimators/utils.cc
        src/geometry/colmap/util/random.cc
        src/geometry/colmap/optim/random_sampler.cc
        src/geometry/colmap/optim/combination_sampler.cc
        src/geometry/colmap/optim/support_measurement.cc

        src/optimization/ba_solver.cc
        src/mapper/incremental_mapper.cc
        src/utility/io_ecim.cc
        )

target_include_directories(xrsfm
        PUBLIC
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/src/geometry/colmap
        ${CMAKE_SOURCE_DIR}/3rdparty/SiftGPU
        ${CMAKE_SOURCE_DIR}/3rdparty/apriltag
        ${EIGEN3_INCLUDE_DIRS}
        )

if(XRPRIMER_ENABLED)
    target_link_libraries(xrsfm
        stdc++fs
        XRPrimer::xrprimer
        ${OpenCV_LIBS}
        ${GTEST_LIBRARIES}
        sift_gpu
        )
else ()
    target_link_libraries(xrsfm
        stdc++fs
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        sift_gpu
        )
endif()

add_executable(run_matching src/run_matching.cc)
target_link_libraries(run_matching xrsfm)

add_executable(run_reconstruction src/run_reconstruction.cc)
target_link_libraries(run_reconstruction xrsfm)

add_executable(run_triangulation src/run_triangulation.cc)
target_link_libraries(run_triangulation xrsfm)

add_executable(unpack_collect_data src/unpack_collect_data.cc)
target_link_libraries(unpack_collect_data xrsfm)

add_executable(estimate_scale src/estimate_scale.cc)
target_link_libraries(estimate_scale xrsfm apriltag)

add_executable(rec_1dsfm src/rec_1dsfm.cc)
target_link_libraries(rec_1dsfm xrsfm)

add_executable(rec_kitti src/rec_kitti.cc)
target_link_libraries(rec_kitti xrsfm)
