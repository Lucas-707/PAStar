project(pastar)

cmake_minimum_required (VERSION 2.6)
enable_language(CUDA)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories("inc")
file(GLOB SOURCES "src/*")
add_executable(pastar ${SOURCES})

# Find CUDA
find_library(CUDART_LIBRARY cudart ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(pastar ${Boost_LIBRARIES})

find_package(MPI REQUIRED)
# IF(MPI_CXX_FOUND)
#         INCLUDE_DIRECTORIES(${MPI_CXX_INCLUDE_PATH})
#         LIST(APPEND SCR_EXTERNAL_LIBS ${MPI_CXX_LIBRARIES})
# ENDIF(MPI_CXX_FOUND)
include_directories(SYSTEM ${MPI_INCLUDE_PATH})
target_link_libraries(pastar ${MPI_C_LIBRARIES})
include_directories(SYSTEM ${MPI_CXX_INCLUDE_PATH})
target_link_libraries(pastar ${MPI_CXX_LIBRARIES})

