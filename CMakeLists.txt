cmake_minimum_required(VERSION 3.16)
project(gpod)

include(CheckCompilerFlag)
math(EXPR stack_size "100*1024*1024")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    if(HAS_STACK_OPTION)
        set(STACK_OPTION "-Wl,--stack,${stack_size}")
    endif()
elseif(MSVC)
    check_compiler_flag("/STACK:${stack_size}" HAS_STACK_OPTION)
    if(HAS_STACK_OPTION)
        set(STACK_OPTION "/STACK:${stack_size}")
    endif()
endif()

if(STACK_OPTION)
    add_compile_options(${STACK_OPTION})  # For compilers that accept this in compile options.
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${STACK_OPTION}")  # For linkers.
endif()

add_subdirectory(lib/sofa)
add_subdirectory(lib/nrlmsise)
add_subdirectory(lib/openblas)
add_subdirectory(lib/gsl)
add_subdirectory(lib/boost)

include_directories(${PROJECT_BINARY_DIR})
include_directories(src)
include_directories(lib)
include_directories(lib/sofa/src)
include_directories(lib/nrlmsise)

link_directories(lib)

aux_source_directory(src SRC_GPOD)

if(WIN32)
    set(LIBS Winmm)
else()
    set(LIBS m pthread)
endif()

add_library(gpod ${SRC_GPOD})
target_link_libraries(gpod ${LIBS} m openblas gsl sofa nrlmsise pthread odeint gfortran quadmath)

add_executable(test_podflt test_podflt.c)
target_link_libraries(test_podflt gpod)








