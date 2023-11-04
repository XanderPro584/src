cmake_minimum_required(VERSION 3.8)
project(turtlesim_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_COMPILER /usr/bin/gcc)  # Replace with the actual path to your C compiler
set(CMAKE_CXX_COMPILER /usr/bin/g++) 

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#Hello I am under the water

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
