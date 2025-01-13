cmake_minimum_required(VERSION 3.8)
project(common)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

    add_compile_options(-Wall -Wextra -Wpedantic )
endif()

find_package(PCL REQUIRED)
find_package(common_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include ${PCL_INCLUDE_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/math.cpp
    src/visualization.cpp
    src/msg.cpp
    src/common.cpp
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_find_build_dependencies()

# target_link_libraries(${PROJECT_NAME}
#     ${PLACEHOLDER}
# )

ament_auto_package()
