cmake_minimum_required(VERSION 3.8)
project(common_msgs)

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
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_find_build_dependencies()

set(MESSAGES
    "msg/Cone.msg"
    "msg/ConeArray.msg"
    "msg/ConePair.msg"
    "msg/ConePairArray.msg"
)

# set(SERVICES
#     "srv/Placeholder.srv"
# )

rosidl_generate_interfaces(${PROJECT_NAME}
    ${MESSAGES}
    # ${SERVICES}
    DEPENDENCIES builtin_interfaces std_msgs
)

# ament_export_dependencies(rosidl_default_runtime)
# ament_package()
ament_auto_package(
    INSTALL_TO_SHARE
)

install(FILES DESTINATION share/${PROJECT_NAME})
