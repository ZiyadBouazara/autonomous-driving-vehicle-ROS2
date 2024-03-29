cmake_minimum_required(VERSION 3.5)
project(design3_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Launch files
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

# Config files
install(DIRECTORY
	config	
	DESTINATION share/${PROJECT_NAME}/
)

# Urdf files
install(DIRECTORY
	urdf
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()
