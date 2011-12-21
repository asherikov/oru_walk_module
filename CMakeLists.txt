cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

use(NAOQI-PLUGINS-TOOLS)


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

# set path to directories containing libraries
link_directories( ${PROJECT_SOURCE_DIR}/smpc_solver/lib/ )
link_directories( ${PROJECT_SOURCE_DIR}/nao_igm/lib/ )


create_module( oru_module )
file (GLOB ORU_MODULE_SRC "${PROJECT_SOURCE_DIR}/src/*.cpp")
configure_src_module(oru_module ${ORU_MODULE_SRC})


# link to libraries and specify path to .h files
target_link_libraries( oru_module wmg smpc_solver naoigm)


# set path to directories containing headers
include_directories(oru_module "${PROJECT_SOURCE_DIR}/smpc_solver/include")
include_directories(oru_module "${PROJECT_SOURCE_DIR}/nao_igm/include")


use_lib(oru_module ALCOMMON ALMEMORY_FAST_ACCESS)
