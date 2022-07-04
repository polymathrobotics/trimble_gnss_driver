cmake_minimum_required(VERSION 2.8.3)
project(trimble_gnss_driver)


find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)


catkin_python_setup()


catkin_package(
#  CATKIN_DEPENDS message_runtime
)


###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
  scripts/gsof_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


foreach (dir launch )
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}/)
endforeach(dir)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_polytunnel_navigation_actions.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
