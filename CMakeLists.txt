cmake_minimum_required(VERSION 3.10)
project(box_measurement_cpp_enhanced_depth LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(box_measure_enhanced src/box_measure_enhanced.cpp)

# Preferred path: build this directory as an extra subdirectory inside OrbbecSDK_v2.
if(TARGET ob::${OB_SDK_LIB_NAME})
    target_link_libraries(box_measure_enhanced PRIVATE ob::${OB_SDK_LIB_NAME} ${OpenCV_LIBS})
else()
    # Standalone path: pass -DOrbbecSDK_ROOT=/path/to/OrbbecSDK_v2 after building/installing the SDK.
    set(OrbbecSDK_ROOT "" CACHE PATH "Path to OrbbecSDK_v2 source/build tree")
    if(NOT OrbbecSDK_ROOT)
        message(FATAL_ERROR "Set OrbbecSDK_ROOT, or add this package as a subdirectory of OrbbecSDK_v2.")
    endif()

    get_filename_component(OrbbecSDK_ROOT "${OrbbecSDK_ROOT}" ABSOLUTE)
    message(STATUS "Using OrbbecSDK_ROOT=${OrbbecSDK_ROOT}")

    set(ORBBEC_SDK_INCLUDE_DIRS
        "${OrbbecSDK_ROOT}/include"
        "${OrbbecSDK_ROOT}/build/src/generated"
        "${OrbbecSDK_ROOT}/build/include"
    )
    target_include_directories(box_measure_enhanced PRIVATE ${ORBBEC_SDK_INCLUDE_DIRS})

    set(ORBBEC_SDK_SEARCH_PATHS
        "${OrbbecSDK_ROOT}/lib"
        "${OrbbecSDK_ROOT}/lib/linux_arm64"
        "${OrbbecSDK_ROOT}/build/lib"
        "${OrbbecSDK_ROOT}/build/lib/linux_arm64"
        "${OrbbecSDK_ROOT}/build/linux_arm64/lib"
        "${OrbbecSDK_ROOT}/build/bin"
        "${OrbbecSDK_ROOT}/build/bin/linux_arm64"
        "${OrbbecSDK_ROOT}/build"
    )

    find_library(ORBBEC_SDK_LIB
        NAMES OrbbecSDK
        PATHS ${ORBBEC_SDK_SEARCH_PATHS}
        NO_DEFAULT_PATH
    )

    if(NOT ORBBEC_SDK_LIB)
        file(GLOB_RECURSE ORBBEC_SDK_LIB_CANDIDATES
            "${OrbbecSDK_ROOT}/libOrbbecSDK.so"
            "${OrbbecSDK_ROOT}/libOrbbecSDK.so.*"
            "${OrbbecSDK_ROOT}/OrbbecSDK.lib"
            "${OrbbecSDK_ROOT}/OrbbecSDK.dll"
        )
        list(SORT ORBBEC_SDK_LIB_CANDIDATES)
        list(LENGTH ORBBEC_SDK_LIB_CANDIDATES ORBBEC_SDK_LIB_CANDIDATE_COUNT)
        if(ORBBEC_SDK_LIB_CANDIDATE_COUNT GREATER 0)
            list(GET ORBBEC_SDK_LIB_CANDIDATES 0 ORBBEC_SDK_LIB)
        endif()
    endif()

    if(NOT ORBBEC_SDK_LIB)
        message(STATUS "Searched paths:")
        foreach(path ${ORBBEC_SDK_SEARCH_PATHS})
            message(STATUS "  ${path}")
        endforeach()
        message(FATAL_ERROR "Could not find libOrbbecSDK.so. Build OrbbecSDK_v2 first, set OrbbecSDK_ROOT to the SDK root, or build this package inside the SDK tree.")
    endif()
    message(STATUS "Using OrbbecSDK library: ${ORBBEC_SDK_LIB}")

    target_link_libraries(box_measure_enhanced PRIVATE ${ORBBEC_SDK_LIB} ${OpenCV_LIBS})

    get_filename_component(ORBBEC_SDK_LIB_DIR "${ORBBEC_SDK_LIB}" DIRECTORY)
    set_target_properties(box_measure_enhanced PROPERTIES
        BUILD_RPATH "${ORBBEC_SDK_LIB_DIR}"
        INSTALL_RPATH "${ORBBEC_SDK_LIB_DIR}"
    )
endif()

if(UNIX)
    target_link_libraries(box_measure_enhanced PRIVATE pthread)
endif()

install(TARGETS box_measure_enhanced RUNTIME DESTINATION bin)
