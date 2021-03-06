cmake_minimum_required(VERSION 2.8.3)

project(nuc)

find_package(LIBGP)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -Wall") # For G++ <4.7; also appears to be supported in G++ 4.7+ as alias to c++11

set(NUC_LIBRARIES GL GLU glut gp)

set(NUC_SOURCES
   # src/dubins.cpp
    src/NUC.cpp
    src/NUC_node.cpp
    src/CNode.cpp
    #src/DepthFirstStrategy.cpp
    #src/ShortCutStrategy.cpp
    #src/LawnmowerStrategy.cpp
    #src/BudgetedStrategy.cpp
    src/MAV.cpp
    src/InterestingnessSensor.cpp
    #src/slic.cpp
    #src/SuperPixelFeatures.cpp
    src/NUCParam.cpp
   # src/GNode.cpp
   # src/PathOptimization.cpp
    #src/HuskyInterface.cpp
    #src/TestStrategy.cpp
    #src/HilbertStrategy.cpp
   # src/HilbertOptimization.cpp
    #src/Trajectory.cpp
    src/SearchCoverageStrategy.cpp
    src/TSP.cpp
    src/TargetPolygon.cpp
    src/GraphComponents.cpp
    #src/Knapsack.cpp
    src/ScalarField.cpp
    src/TargetTour.cpp
    src/CompoundTarget.cpp
    include/ScalarField.h
    include/CompoundTarget.h
    include/TargetTour.h
    include/Knapsack.h
    include/GraphComponents.h
    include/TargetPolygon.h
    include/TSP.h
    include/SearchCoverageStrategy.h
    #include/dubins.h
    #include/HilbertStrategy.h
    #include/TestStrategy.h
    #include/BudgetedStrategy.h
    #include/Trajectory.h
    #include/HuskyInterface.h
    #include/SuperPixelFeatures.h
    #include/slic.h
    include/InterestingnessSensor.h
    include/NUC.h
    include/CNode.h
    #include/DepthFirstStrategy.h
    #include/ShortCutStrategy.h
    #include/LawnmowerStrategy.h
    include/TraversalStrategy.h
    #include/HilbertOptimization.h
    include/MAV.h
    include/NUCParam.h
    #include/GNode.h
    #include/PathOptimization.h
)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  nav_msgs
  message_generation
  image_transport
  cv_bridge
  pelican_ctrl
  )

#generate_dynamic_reconfigure_options(
#  cfg/UEyeCam.cfg
#  )

add_service_files(FILES
   periodic.srv  
   start_log.srv
)

generate_messages()


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy  geometry_msgs  std_msgs  nav_msgs  message_generation  image_transport  cv_bridge  pelican_ctrl  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBGP_INCLUDE_DIRS}
  )

  
add_executable(NUC ${NUC_SOURCES})
target_link_libraries(NUC  ${NUC_LIBRARIES} ${catkin_LIBRARIES} ${LIBGP_LIBRARIES})

add_executable(ImagePublisher src/ImagePublisher.cpp)
target_link_libraries(ImagePublisher ${catkin_LIBRARIES})

add_executable(PlayLog src/PlayLog.cpp include/PlayLog.h)
target_link_libraries(PlayLog ${NUC_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(stitcher src/stitching_detailed.cpp)
#target_link_libraries(stitcher ${catkin_LIBRARIES})

#install(TARGETS ${UEYECAM_LIB_NAME} ${UEYECAM_NODELET_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )

#install(FILES nodelet_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  )

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  )

