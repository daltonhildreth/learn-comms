option(GLFW_BUILD_DOCS OFF)
option(GLFW_BUILD_EXAMPLES OFF)
option(GLFW_BUILD_TESTS OFF)
add_subdirectory(lib/glfw)

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
    -DPROJECT_DIR="${PROJECT_SOURCE_DIR}/"
    -DPROJECT_SRC_DIR="${PROJECT_SOURCE_DIR}/enginee/"
)

add_definitions(-DPRM_DEBUG)

if (${CMAKE_BUILD_TYPE} MATCHES Debug)
    add_definitions(-DDEBUG)
endif()
if(NOT WIN32)
    set(DLLOAD dl)
endif()

#to avoid compiler warnings i seperately build these
add_library(glad ${PROJECT_SOURCE_DIR}/enginee/lib/glad/glad.c)
target_include_directories(glad PUBLIC lib/glad/)
add_library(stb_image ${PROJECT_SOURCE_DIR}/enginee/lib/stb_image.cpp)
target_include_directories(stb_image PUBLIC lib/)

foreach(CONFIG nocomm_norender nocomm_render comm_norender comm_render)
    add_executable(${CONFIG} ${PROJECT_SRC} ${PROJECT_LIB})
    target_link_libraries(${CONFIG} glad stb_image glfw ${DLLOAD})

    if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        #Do not use warnings in MSVC because they are typically != to gcc/clang's.
        #Normally more warnings is always better, but I don't enforce Windows
        #compatability enough to care about its warnings. :( Also, I don't have a
        #clean way of ignoring warnings from /lib files.
        target_compile_options(${CONFIG} PUBLIC /W0 /std:c++latest)
        target_compile_definitions(${CONFIG} PUBLIC _CRT_SECURE_NO_WARNINGS)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        #alternate for clang & gcc: -Wall -Wextra -pedantic -std=c++17 -Werror
        #Keep the following, though; it took forever to compile.
        #TODO: fix strict overflow=5 warnings in release.
        target_compile_options(
            ${CONFIG}
            PUBLIC
            -Wall -Wextra -Wpedantic -std=c++17 -Werror -Wfatal-errors
            -Wold-style-cast -Wcast-align -Wcast-qual -Wuseless-cast
            -Wconversion -Wsign-conversion -Wsign-promo -Wsign-promo
            -Wdisabled-optimization -Wno-strict-overflow -Wabi
            -Wfloat-equal -Wlogical-op
            -Wmissing-declarations -Wmissing-include-dirs -Wundef
            -Wformat=2 -Wzero-as-null-pointer-constant -Wstrict-null-sentinel
            -Wredundant-decls -Wshadow -Wunused
            -Woverloaded-virtual -Wctor-dtor-privacy
        )
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Intel")
        #???
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(
            ${CONFIG}
            PUBLIC
            -Wall -Wextra -Wpedantic -std=c++17 -Werror
        )
    endif()

    set_target_properties(
        ${CONFIG} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
    target_include_directories(${CONFIG} PUBLIC src/)
    target_include_directories(
        ${CONFIG}
        SYSTEM PUBLIC
        lib/glad/ lib/glfw/include/ lib/glm/ lib/
    )
endforeach(CONFIG)

target_compile_definitions(nocomm_norender PUBLIC NO_COMM NO_RENDER)
target_compile_definitions(nocomm_render PUBLIC NO_COMM)
target_compile_definitions(comm_norender PUBLIC NO_RENDER)
