#include "ros/ros.h"
#include "Eigen/Dense"
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>



void get_trajectory( Eigen::VectorXd com_p, Eigen::VectorXd com_dp, Eigen::VectorXd dcom_p, Eigen::Vector3d eeBLpos, const Eigen::Vector3d eeBRpos, 
    Eigen::Vector3d eeFLpos, Eigen::Vector3d eeFRpos, int gait_flag, float duration, towr::SplineHolder & solution, towr::NlpFormulation & formulation_ );


