cmake_minimum_required(VERSION 3.8)
project(common)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(common_msgs REQUIRED)

# find_package(Lemon REQUIRED)
# set(LEMON_INCLUDE_DIRS "/usr/include/lemon")
find_package(PCL REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

include_directories(include
    ${PCL_INCLUDE_DIRS}
)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/math.cpp
    src/msg.cpp
    src/viz.cpp
    src/cone.cpp
    src/cone_graph.cpp
    src/cones.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
)

ament_auto_package()

install(FILES DESTINATION share/${PROJECT_NAME})
