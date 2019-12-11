cmake_minimum_required(VERSION 2.8.3)
project(mocap_lib)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  tf2_ros
  tf2_eigen)

find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf2_ros tf2_eigen
  DEPENDS TinyXML
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${TinyXML_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/marker_lib.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${TinyXML_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)

catkin_add_gtest(mocap_lib_test test/geometry_tests.cpp)
target_link_libraries(mocap_lib_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

endif()
