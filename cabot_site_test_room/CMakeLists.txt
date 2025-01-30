cmake_minimum_required(VERSION 3.5)
project(cabot_site_test_room)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps/)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models/)
install(DIRECTORY queue/ DESTINATION share/${PROJECT_NAME}/queue/)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds/)
install(DIRECTORY server_data/ DESTINATION share/${PROJECT_NAME}/server_data/)

ament_package()
