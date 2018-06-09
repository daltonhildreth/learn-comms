cmake_minimum_required(VERSION 3.5)
project(gg-engine)

option(GLFW_BUILD_DOCS OFF)
option(GLFW_BUILD_EXAMPLES OFF)
option(GLFW_BUILD_TESTS OFF)
add_subdirectory(lib/glfw)

#option(ASSIMP_BUILD_ASSIMP_TOOLS OFF)
#option(ASSIMP_BUILD_SAMPLES OFF)
#option(ASSIMP_BUILD_TESTS OFF)
#add_definitions(
#    -DASSIMP_BUILD_NO_EXPORT
#    -DASSIMP_BUILD_NO_AMF_IMPORTER
#    -DASSIMP_BUILD_NO_3DS_IMPORTER
#    -DASSIMP_BUILD_NO_AC_IMPORTER
#    -DASSIMP_BUILD_NO_ASE_IMPORTER
#    -DASSIMP_BUILD_NO_ASSBIN_IMPORTER
#    -DASSIMP_BUILD_NO_ASSXML_IMPORTER
#    -DASSIMP_BUILD_NO_B3D_IMPORTER
#    -DASSIMP_BUILD_NO_BVH_IMPORTER
#    -DASSIMP_BUILD_NO_COLLADA_IMPORTER
#    -DASSIMP_BUILD_NO_DXF_IMPORTER
#    -DASSIMP_BUILD_NO_CSM_IMPORTER
#    -DASSIMP_BUILD_NO_HMP_IMPORTER
#    -DASSIMP_BUILD_NO_IRRMESH_IMPORTER
#    -DASSIMP_BUILD_NO_IRR_IMPORTER
#    -DASSIMP_BUILD_NO_LWO_IMPORTER
#    -DASSIMP_BUILD_NO_LWS_IMPORTER
#    -DASSIMP_BUILD_NO_MD2_IMPORTER
#    -DASSIMP_BUILD_NO_MD3_IMPORTER
#    -DASSIMP_BUILD_NO_MD5_IMPORTER
#    -DASSIMP_BUILD_NO_MDC_IMPORTER
#    -DASSIMP_BUILD_NO_MDL_IMPORTER
#    -DASSIMP_BUILD_NO_NFF_IMPORTER
#    -DASSIMP_BUILD_NO_NDO_IMPORTER
#    -DASSIMP_BUILD_NO_OFF_IMPORTER
#    -DASSIMP_BUILD_NO_OBJ_IMPORTER
#    -DASSIMP_BUILD_NO_OGRE_IMPORTER
#    -DASSIMP_BUILD_NO_OPENGEX_IMPORTER
#    -DASSIMP_BUILD_NO_PLY_IMPORTER
#    -DASSIMP_BUILD_NO_MS3D_IMPORTER
#    -DASSIMP_BUILD_NO_COB_IMPORTER
#    -DASSIMP_BUILD_NO_BLEND_IMPORTER
#    -DASSIMP_BUILD_NO_IFC_IMPORTER
#    -DASSIMP_BUILD_NO_XGL_IMPORTER
#    -DASSIMP_BUILD_NO_FBX_IMPORTER
#    -DASSIMP_BUILD_NO_Q3D_IMPORTER
#    -DASSIMP_BUILD_NO_Q3BSP_IMPORTER
#    -DASSIMP_BUILD_NO_RAW_IMPORTER
#    -DASSIMP_BUILD_NO_SIB_IMPORTER
#    -DASSIMP_BUILD_NO_SMD_IMPORTER
#    -DASSIMP_BUILD_NO_STL_IMPORTER
#    -DASSIMP_BUILD_NO_TERRAGEN_IMPORTER
#    -DASSIMP_BUILD_NO_3D_IMPORTER
#    -DASSIMP_BUILD_NO_X_IMPORTER
#    -DASSIMP_BUILD_NO_X3D_IMPORTER
#    -DASSIMP_BUILD_NO_GLTF_IMPORTER
#    -DASSIMP_BUILD_NO_3MF_IMPORTER
#    -DASSIMP_BUILD_NO_MMD_IMPORTER
#)
#add_subdirectory(lib/assimp)

#catch, glm, and stb_image are header only libraries

include_directories(src/)
include_directories(SYSTEM
#    lib/assimp/include/
    lib/glad/
    lib/glfw/include/
    lib/glm/
    lib/
)

file(GLOB_RECURSE PROJECT_SRC
    src/*.c
    src/*.h
    src/*.cpp
    src/*.hpp
)
file(GLOB_RECURSE PROJECT_RES
    res/*
)
file(GLOB_RECURSE PROJECT_TEST
    test/*.cpp
    test/*.c
)
file(GLOB_RECURSE PROJECT_DOC
    doc/*
    .gitignore
    CMakeLists.txt
    README.md
)

source_group("test" FILES ${PROJECT_TEST})
source_group("doc" FILES ${PROJECT_DOC})
source_group("res" FILES ${PROJECT_RES})
source_group("src" FILES ${PROJECT_SRC})
source_group("lib" FILES ${PROJECT_LIB})

add_definitions(
    -DGLFW_INCLUDE_NONE
    -DPROJECT_SRC_DIR=\"${PROJECT_SOURCE_DIR}\"
)

add_definitions(-DSTOP_COLLISION)
add_definitions(-DSCENE_N=5)
if (NO_COMM)
    add_definitions(-DNO_COMM)
endif()
if (NO_RENDER)
    add_definitions(-DNO_RENDER)
endif()
add_definitions(-DDATA_DIR=\"${PROJECT_SOURCE_DIR}/..\"\"/data\")
add_definitions(-DPRE_R=\"\")#"\no\"
#add_definitions(-DPRM_DEBUG)

if (${CMAKE_BUILD_TYPE} MATCHES Debug)
    add_definitions(-DDEBUG)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    #Do not use warnings in MSVC because they are typically != to gcc/clang's.
    #Normally more warnings is always better, but I don't enforce Windows
    #compatability enough to care about its warnings. :( Also, I don't have a
    #clean way of ignoring warnings from /lib files.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W0 /std:c++latest")
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    #alternate for clang & gcc: -Wall -Wextra -pedantic -std=c++17 -Werror
    #Keep the following, though; it took forever to compile.
    #TODO: fix strict overflow=5 warnings in release.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
        -Wall -Wextra -Wpedantic -std=c++17 -Werror -Wfatal-errors\
        -Wold-style-cast -Wcast-align -Wcast-qual -Wuseless-cast \
        -Wconversion -Wsign-conversion -Wsign-promo -Wsign-promo \
        -Wdisabled-optimization -Wno-strict-overflow -Wabi \
        -Wfloat-equal -Wlogical-op \
        -Wmissing-declarations -Wmissing-include-dirs -Wundef \
        -Wformat=2 -Wzero-as-null-pointer-constant -Wstrict-null-sentinel \
        -Wredundant-decls -Wshadow -Wunused \
        -Woverloaded-virtual -Wctor-dtor-privacy")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
    # DUno the flags
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
        -Wall -Wextra -Wpedantic -std=c++17 -Werror")
endif()
if(NOT WIN32)
    set(DLLOAD dl)
endif()

#to avoid compiler warnings i seperately build these
add_library(glad ${PROJECT_SOURCE_DIR}/lib/glad/glad.c)
add_library(stb_image ${PROJECT_SOURCE_DIR}/lib/stb_image.cpp)

add_executable(${PROJECT_NAME} ${PROJECT_SRC} ${PROJECT_LIB})

target_link_libraries(
    ${PROJECT_NAME}
    #assimp
    glad
    stb_image
    glfw ${GLFW_LIBRARIES}
    ${DLLOAD}
    #catch, glm, stb_image are header only
)

set_property(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME}
)

set_target_properties(
    ${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
