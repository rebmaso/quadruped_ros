cmake_minimum_required(VERSION 3.0.2)
project(popt)

# add idyntree folder to cmake prefix path
list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/install/usr/local/lib/cmake/iDynTree")

# message(FATAL_ERROR "${CMAKE_PREFIX_PATH}")

find_package(iDynTree REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  geometry_msgs
  roscpp
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf
  roslib
  tf_conversions
  ifopt
  tf2_msgs
  tf
)

find_library(ZBAR_LIBRARIES NAMES zbar)

find_package( ifopt REQUIRED )
include_directories( include )
include_directories( /usr/include/eigen3 )
include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_package()

SET( ALGLIB_SRC 
  src/alglib/alglibinternal.cpp
  src/alglib/alglibmisc.cpp
  src/alglib/ap.cpp
  src/alglib/linalg.cpp
  src/alglib/optimization.cpp
  src/alglib/solvers.cpp
)

SET ( TOWR_SRC 
  src/towr/nlp_formulation.cc
  src/towr/parameters.cc
  src/towr/nodes_variables.cc
  src/towr/nodes_variables_all.cc
  src/towr/nodes_variables_phase_based.cc
  src/towr/phase_durations.cc
  src/towr/robot_model.cc
  src/towr/dynamic_model.cc
  src/towr/single_rigid_body_dynamics.cc
  src/towr/time_discretization_constraint.cc
  src/towr/base_motion_constraint.cc
  src/towr/terrain_constraint.cc
  src/towr/swing_constraint.cc
  src/towr/force_constraint.cc
  src/towr/total_duration_constraint.cc
  src/towr/dynamic_constraint.cc
  src/towr/range_of_motion_constraint.cc
  src/towr/spline_acc_constraint.cc
  src/towr/linear_constraint.cc
  src/towr/node_cost.cc
  src/towr/soft_constraint.cc
  src/towr/gait_generator.cc
  src/towr/monoped_gait_generator.cc
  src/towr/biped_gait_generator.cc
  src/towr/quadruped_gait_generator.cc
  src/towr/height_map_examples.cc
  src/towr/height_map.cc
  src/towr/state.cc
  src/towr/polynomial.cc
  src/towr/spline.cc
  src/towr/phase_spline.cc
  src/towr/node_spline.cc
  src/towr/nodes_observer.cc
  src/towr/spline_holder.cc
  src/towr/euler_converter.cc
  src/towr/phase_durations_observer.cc
)


add_library( lopt src/lopt.cpp ${ALGLIB_SRC} )
target_link_libraries( lopt ${catkin_LIBRARIES}  )

add_library( topt src/topt.cpp ${TOWR_SRC} ) 
target_link_libraries( topt ${catkin_LIBRARIES})

add_executable(main src/client/Main.cpp  src/client/Planner.cpp src/client/Controller.cpp)
target_link_libraries( main lopt topt zbar ${catkin_LIBRARIES}  ${iDynTree_LIBRARIES})

# # w/o planner
# add_executable(main src/client/Main.cpp src/client/Controller.cpp)
# target_link_libraries( main lopt topt ${catkin_LIBRARIES}  ${iDynTree_LIBRARIES})

add_executable(converter src/client/Converter.cc)
target_link_libraries(converter ${catkin_LIBRARIES} )
