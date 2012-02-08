cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")



####################################
# Settings
####################################
set (CMAKE_SKIP_ASSEMBLY_SOURCE_RULES ON)
set (CMAKE_SKIP_PREPROCESSED_SOURCE_RULES ON)
set (CMAKE_VERBOSE_MAKEFILE ON) # print full commands during compilation
set (CMAKE_RULE_MESSAGES OFF CACHE BOOL "")


####################################
# Other parameters
####################################

find_path(EIGEN_INCLUDE_DIRS "Eigen/Core" PATHS "${SDK_DIR}/include" PATH_SUFFIXES "eigen2")
if (EIGEN_INCLUDE_DIRS)
    include_directories ("${EIGEN_INCLUDE_DIRS}")
endif (EIGEN_INCLUDE_DIRS)


project ( oru_walk )
file (GLOB ORU_WALK_SRC "${PROJECT_SOURCE_DIR}/src/*.cpp")
qi_create_lib(oru_walk ${ORU_WALK_SRC})


# link to libraries
target_link_libraries(oru_walk
    "${PROJECT_SOURCE_DIR}/smpc_solver/lib/libwmg.a" 
    "${PROJECT_SOURCE_DIR}/smpc_solver/lib/libsmpc_solver.a" 
    "${PROJECT_SOURCE_DIR}/nao_igm/lib/libnaoigm.a")


# set path to directories containing headers
include_directories(oru_walk "${PROJECT_SOURCE_DIR}/smpc_solver/include")
include_directories(oru_walk "${PROJECT_SOURCE_DIR}/nao_igm/include")


qi_use_lib(oru_walk ALCOMMON ALMEMORYFASTACCESS)
