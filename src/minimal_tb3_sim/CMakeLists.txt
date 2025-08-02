cmake_minimum_required(VERSION 3.8)
project(minimal_tb3_sim)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY configs launch worlds urdf 
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY models/
        DESTINATION share/${PROJECT_NAME}/models)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
