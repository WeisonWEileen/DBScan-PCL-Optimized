cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/color.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/cmake/functions.cmake")

set(CMAKE_BUILD_TYPE_INIT "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(app VERSION 1.0.6)

fetch_project(
    NAME cloudparse
    URL https://github.com/danielTobon43/cloudparse/archive/v0.2.1.tar.gz
    )
fetch_project(
        NAME argparse
        URL https://github.com/p-ranav/argparse/archive/v2.6.tar.gz
)
        
message("\n" "${BoldCyan}=========================================")
message("${BoldCyan}Project: ${PROJECT_NAME} ")
message("${BoldCyan}=========================================")



find_package(PCL  REQUIRED QUIET COMPONENTS visualization)
find_package(VTK  REQUIRED QUIET COMPONENTS vtkRenderingContextOpenGL2)

add_subdirectory(dbscan)

set(MAIN_SOURCE "main.cpp")

add_executable(${PROJECT_NAME} ${MAIN_SOURCE})


target_include_directories(${PROJECT_NAME} PRIVATE ${VTK_INCLUDE_DIRS} include)

target_link_libraries(${PROJECT_NAME} argparse dbscan cloudparse ${VTK_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} Boost::filesystem Boost::system vtkInteractionStyle)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cpp
    -mmmx
    -msse
    -msse2
    -msse3
    -mssse3
    -msse4.2
    -msse4.1
    -mno-sse4a
    -mno-avx
    -mno-avx2
    -mno-fma
    -mno-fma4
    -mno-f16c
    -mno-xop
    -mno-bmi
    -mno-bmi2
    -mrdrnd
    -mno-3dnow
    -mlzcnt
    -mfsgsbase
    -mpclmul
)
install( TARGETS ${PROJECT_NAME}
RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# # Automatically find the include directory for argparse
find_path(ARGPARSE_INCLUDE_DIR "argparse/argparse.hpp" PATHS ${argparse_SOURCE_DIR}/include)