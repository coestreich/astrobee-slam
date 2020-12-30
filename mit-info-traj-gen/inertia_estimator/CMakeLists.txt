project(inertia_estimator)

add_message_files(
   FILES
   params.msg
 )

 generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  # CATKIN_DEPENDS roscpp rospy std_msgs message_generation message_runtime tf 
)



include_directories(
        include ${catkin_INCLUDE_DIRS}
        include ${EIGEN3_INCLUDE_DIRS}
)

 

add_executable(inertia_estimator src/main.cpp)

add_dependencies(inertia_estimator
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
         ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(inertia_estimator ${catkin_LIBRARIES} ${eigen_LIBRARIES})

install(TARGETS inertia_estimator DESTINATION bin)