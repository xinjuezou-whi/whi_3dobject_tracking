cmake_minimum_required(VERSION 3.10)

# Define headers and sources
# =============================================================================
FILE(GLOB SOURCES "*.cpp")
FILE(GLOB HEADERS "../include/m3t/*.h")
if (NOT USE_AZURE_KINECT)
    list(FILTER HEADERS EXCLUDE REGEX "azure_kinect_camera.h")
    list(FILTER SOURCES EXCLUDE REGEX "azure_kinect_camera.cpp")
endif()
if (NOT USE_REALSENSE)
    list(FILTER HEADERS EXCLUDE REGEX "realsense_camera.h")
    list(FILTER SOURCES EXCLUDE REGEX "realsense_camera.cpp")
endif()


# Define libraries
# =============================================================================
set(LIBRARIES Eigen3::Eigen OpenGL::GL GLEW::GLEW glfw ${OpenCV_LIBRARIES})
if (UNIX)
    list(APPEND LIBRARIES pthread stdc++fs)
endif ()
if (USE_AZURE_KINECT)
    list(APPEND LIBRARIES k4a::k4a)
endif ()
if (USE_REALSENSE)
    list(APPEND LIBRARIES ${realsense2_LIBRARY})
endif ()

# Define target
# =============================================================================
add_library(m3t ${SOURCES})
target_link_libraries(m3t PUBLIC ${LIBRARIES})
target_include_directories(m3t PUBLIC  
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:include>)
target_include_directories(m3t PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../third_party>
        $<INSTALL_INTERFACE:include>)
set_target_properties(m3t PROPERTIES PUBLIC_HEADER "${HEADERS}")


# Install
# =============================================================================
install(TARGETS m3t
        EXPORT m3t_target
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include/m3t)
install(DIRECTORY ../third_party/
        DESTINATION include)
install(EXPORT m3t_target 
        FILE M3TConfig.cmake
        DESTINATION lib/cmake/M3T)
