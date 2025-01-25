cmake_minimum_required(VERSION 3.16)

project(boids-sim VERSION 1.0
        DESCRIPTION "Boids simulation"
        LANGUAGES CUDA CXX)

# Set C++ and CUDA standards
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CUDA_STANDARD 11)
include(FetchContent)

find_package(CUDA REQUIRED)
find_package(OpenGL REQUIRED)

# Fetching dependencies: glfw, glad, glm
FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        3.3.4
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
        glad
        GIT_REPOSITORY https://github.com/Dav1dde/glad.git
        GIT_TAG        v0.1.34
)
FetchContent_MakeAvailable(glad)

FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG        1.0.1
)
FetchContent_MakeAvailable(glm)

FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.90.9
)
FetchContent_MakeAvailable(imgui)
 
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cu"
)

set(IMGUI_SRC_FILES
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

set(ALL_SRC_FILES
        ${SRC_FILES}
        ${IMGUI_SRC_FILES}
)

add_executable(${CMAKE_PROJECT_NAME} ${ALL_SRC_FILES})

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/resources
        $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>/resources)

# MSVC-specific settings
if(MSVC)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC _CRT_SECURE_NO_WARNINGS)
    target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE legacy_stdio_definitions.lib)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDebug$<$<CONFIG:Debug>:Debug>")
endif()

# Include directories
target_include_directories("${CMAKE_PROJECT_NAME}" PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/include/"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/rendering"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/cpu"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/gpu"
    ${CUDA_INCLUDE_DIRS}
    ${glm_SOURCE_DIR}
    ${imgui_SOURCE_DIR} 
    ${imgui_SOURCE_DIR}/backends
    ${glfw_SOURCE_DIR}/include  # Add GLFW's include directory
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE glfw glad OpenGL::GL)

# CUDA-specific settings
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON  # Enable separable compilation for CUDA
)