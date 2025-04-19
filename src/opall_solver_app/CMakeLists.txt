cmake_minimum_required(VERSION 3.30)
project(opall_solver_app VERSION 1.0.0)

add_executable(opall_solver_app
input_output.cpp
main.cpp
CommandLineParameters.cpp
ConfigParser.cpp
ReportData.cpp
include/input_output.hpp
include/CommandLineParameters.hpp
include/Config.hpp
include/ReportConfig.hpp
include/ConfigParser.hpp
include/ReportData.hpp)

target_compile_definitions(opall_solver_app PRIVATE 
    APP_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    APP_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    APP_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)

set_property(TARGET opall_solver_app PROPERTY CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

target_link_libraries(opall_solver_app PRIVATE opall)

find_package(Eigen3 3.4 REQUIRED)

target_link_libraries(opall_solver_app PRIVATE Eigen3::Eigen)
target_include_directories(opall_solver_app PRIVATE include)

include(GNUInstallDirs)
install(TARGETS opall_solver_app RUNTIME)