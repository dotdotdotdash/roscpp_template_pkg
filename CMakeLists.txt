cmake_minimum_required(VERSION 2.8.3)
project(template_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  dynamic_reconfigure
  tf
)

generate_dynamic_reconfigure_options(
  config/ExampleNode.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES template_pkg
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(example_node src/example_node.cpp)
target_link_libraries(example_node ${catkin_LIBRARIES})
add_dependencies(example_node template_pkg_gencfg tf)

#############
## Testing ##
#############

roslint_cpp()

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(sample_test src/test/sample_test_launch.test src/test/sample_test.cpp)
    target_link_libraries(sample_test ${catkin_LIBRARIES})
endif()

#############
## Install ##
#############
install(DIRECTORY config
                  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS example_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
