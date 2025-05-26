project(opall VERSION 1.0.0)
cmake_minimum_required(VERSION 3.30)

add_library(opall STATIC
Problem.cpp
OptimizationDataContainer.cpp
CostFunctionDataContainer.cpp
IncrementalAlignment.cpp
cost_function_data_creator.cpp
cost_function_data_id_generator.cpp
optimization_data_processor.cpp
create_ceres_solver_options.cpp
evaluators.cpp
geometry.cpp
residual_computation.cpp
fill_problem.cpp
full_covariance_computation.cpp
include/opall/data_types.hpp
include/opall/Problem.hpp
include/opall/SolverSummary.hpp
include/opall/CostFunctionIdentifier.hpp
include/opall/SparseJacobianData.hpp
include/opall/cost_function_data/CostFunctionData.hpp
include/opall/cost_function_data/ObservedPointIn3DCostFunctionData.hpp
include/opall/cost_function_data/ObservedPointIn3DFixedStationCostFunctionData.hpp
include/opall/cost_function_data/PointToPointCostFunctionData.hpp
include/opall/cost_function_data/PointToPointFixedStationCostFunctionData.hpp

include/opall/geometry.hpp
include/opall/residual_computation.hpp
include/opall/fill_problem.hpp
include/opall/full_covariance_computation.hpp

include/opall/OptimizationDataContainer.hpp
include/opall/OptimizationConfig.hpp
include/opall/IncrementalAlignment.hpp
include/opall/ModellingConfig.hpp
include/opall/CostFunctionDataContainer.hpp
include/opall/LossFunctionDescription.hpp

include/opall/cost_function_data_creator.hpp
include/opall/cost_function_data_id_generator.hpp
include/opall/optimization_data_processor.hpp
include/opall/evaluators.hpp

include/cost_functors/ObservedPointIn3DCostFunctor.hpp
include/cost_functors/ObservedPointIn3DFixedStationCostFunctor.hpp
include/cost_functors/PointToPointCostFunctor.hpp
include/cost_functors/PointToPointFixedStationCostFunctor.hpp

include/uncertainty_conversion.hpp
include/create_ceres_solver_options.hpp
)

source_group("cost_function_data" FILES
include/opall/cost_function_data/CostFunctionData.hpp
include/opall/cost_function_data/ObservedPointIn3DCostFunctionData.hpp
include/opall/cost_function_data/ObservedPointIn3DFixedStationCostFunctionData.hpp
include/opall/cost_function_data/PointToPointCostFunctionData.hpp
include/opall/cost_function_data/PointToPointFixedStationCostFunctionData.hpp
)

source_group("cost_functors" FILES
include/cost_functors/ObservedPointIn3DCostFunctor.hpp
include/cost_functors/ObservedPointIn3DFixedStationCostFunctor.hpp
include/cost_functors/PointToPointCostFunctor.hpp
include/cost_functors/PointToPointFixedStationCostFunctor.hpp
)

set_target_properties(opall PROPERTIES PUBLIC_HEADER 
"include/opall/data_types.hpp;include/opall/Problem.hpp;include/opall/SolverSummary.hpp;include/opall/CostFunctionIdentifier.hpp;include/opall/cost_function_data/CostFunctionData.hpp;include/opall/cost_function_data/ObservedPointIn3DCostFunctionData.hpp;include/opall/cost_function_data/ObservedPointIn3DFixedStationCostFunctionData.hpp;include/opall/cost_function_data/PointToPointCostFunctionData.hpp;include/opall/cost_function_data/PointToPointFixedStationCostFunctionData.hpp;include/opall/geometry.hpp;include/opall/residual_computation.hpp;include/opall/fill_problem.hpp;include/opall/OptimizationDataContainer.hpp;include/opall/OptimizationConfig.hpp;include/opall/IncrementalAlignment.hpp;include/opall/ModellingConfig.hpp;include/opall/CostFunctionDataContainer.hpp;include/opall/LossFunctionDescription.hpp;include/opall/cost_function_data_creator.hpp;include/opall/cost_function_data_id_generator.hpp;include/opall/optimization_data_processor.hpp;include/opall/evaluators.hpp")

set_property(TARGET opall PROPERTY CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres 2.1 REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

target_link_libraries(opall PRIVATE Ceres::ceres)
target_link_libraries(opall PRIVATE Eigen3::Eigen)

target_sources(opall PUBLIC FILE_SET HEADERS BASE_DIRS include FILES
include/opall/data_types.hpp
include/opall/Problem.hpp
include/opall/SolverSummary.hpp
include/opall/CostFunctionIdentifier.hpp
include/opall/cost_function_data/CostFunctionData.hpp
include/opall/cost_function_data/ObservedPointIn3DCostFunctionData.hpp
include/opall/cost_function_data/ObservedPointIn3DFixedStationCostFunctionData.hpp
include/opall/cost_function_data/PointToPointCostFunctionData.hpp
include/opall/cost_function_data/PointToPointFixedStationCostFunctionData.hpp
include/opall/geometry.hpp
include/opall/residual_computation.hpp
include/opall/fill_problem.hpp
include/opall/OptimizationDataContainer.hpp
include/opall/OptimizationConfig.hpp
include/opall/IncrementalAlignment.hpp
include/opall/ModellingConfig.hpp
include/opall/CostFunctionDataContainer.hpp
include/opall/LossFunctionDescription.hpp
include/opall/cost_function_data_creator.hpp
include/opall/cost_function_data_id_generator.hpp
include/opall/optimization_data_processor.hpp
include/opall/evaluators.hpp
)

target_include_directories(opall PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

target_compile_definitions(opall PRIVATE 
    LIB_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    LIB_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    LIB_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)

include(GNUInstallDirs)
install(TARGETS opall EXPORT opallTargets FILE_SET HEADERS DESTINATION include PUBLIC_HEADER EXCLUDE_FROM_ALL)
#used EXCLUDE_FROM_ALL not to copy public headers twice to include destination

install(EXPORT opallTargets DESTINATION ${CMAKE_INSTALL_LIBDIR}/opall/cmake NAMESPACE opall::
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/opallConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/opallConfig.cmake"
    INSTALL_DESTINATION lib/cmake/opall
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/opallConfig.cmake" DESTINATION lib/opall/cmake
)


