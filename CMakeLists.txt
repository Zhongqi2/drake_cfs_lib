cmake_minimum_required(VERSION 3.10.2)
project(drake_cmake_installed)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(drake CONFIG REQUIRED)

set(SOURCES 
    src/RobotVisualizer/RobotVisualizer.cpp
    src/KinematicTrajetcoryOptimization/KinematicTrajOpt.cpp
    main.cpp
)

add_executable(trajectory_optimization_lib ${SOURCES})
target_link_libraries(trajectory_optimization_lib drake::drake)
target_include_directories(trajectory_optimization_lib PUBLIC 
    ${CMAKE_SOURCE_DIR}/include/KinematicTrajetcoryOptimization
    ${CMAKE_SOURCE_DIR}/include/RobotVisualizer
)

# ---------------------------------------------------------------------------
# Motion planner library (multiple planning methods) + apps
# ---------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_library(motion_planner STATIC
    src/MotionPlanner/PlanningScene.cpp
    src/MotionPlanner/TrajectoryUtils.cpp
    src/MotionPlanner/MotionPlanner.cpp
    src/MotionPlanner/SimplePlanners.cpp
    src/MotionPlanner/BezierPlanner.cpp
    src/MotionPlanner/RRTConnectPlanner.cpp
    src/MotionPlanner/TrajOptPlanner.cpp
    src/MotionPlanner/PlannerUI.cpp
)
target_include_directories(motion_planner PUBLIC ${CMAKE_SOURCE_DIR}/include/MotionPlanner)
target_link_libraries(motion_planner PUBLIC drake::drake)
target_compile_definitions(motion_planner PUBLIC PROJECT_ROOT="${CMAKE_SOURCE_DIR}")

# Interactive UI: browser controls + Drake Meshcat visualizer
add_executable(planner_ui apps/planner_ui.cpp)
target_link_libraries(planner_ui motion_planner)

# Headless comparison of all planners on random queries
add_executable(planner_benchmark apps/planner_benchmark.cpp)
target_link_libraries(planner_benchmark motion_planner)
