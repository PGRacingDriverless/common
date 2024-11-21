cmake_minimum_required(VERSION 3.5)
project(common)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# OpenCV
# find_package(OpenCV 4.9 REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/${PROJECT_NAME}.cpp
)

# target_link_libraries(
#     ${PROJECT_NAME}
#     ${OpenCV_LIBS}
# )

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
