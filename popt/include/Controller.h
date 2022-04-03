#ifndef INCLUDE_CONTROLLER_HPP_
#define INCLUDE_CONTROLLER_HPP_

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <map>
#include <thread>
#include <unistd.h>
#include <unordered_map>

#include <ros/package.h>
#include "std_msgs/Float64MultiArray.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include <std_srvs/Empty.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>
#include "gazebo_msgs/ContactsState.h"
#include "rosgraph_msgs/Clock.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/JointState.h"
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>
#include "topt.h"
#include "lopt.h"

using namespace std;

class Controller {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // to avoid alignment issues, just in case
        Controller() = delete;
        Controller(ros::NodeHandle& nh);
        ~Controller();
        // void gazeboclockCallback (const rosgraph_msgs::Clock & msg);
        void jointStateCallback(const sensor_msgs::JointState & msg);
        void modelStateCallback(const gazebo_msgs::ModelStates & msg);
        void navCallback(const geometry_msgs::Twist& msg);
        void ctrl_loop();
        void publish_cmd(  Eigen::VectorXd tau  );
        void eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr);
        void eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl);
        void eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr);
        void eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl);
        void createrobot(std::string modelFile);
        // Compute the Jacobian
        void  computeJac();
        void  ComputeJaclinear();
        // Compute matrix transformation T needed to recompute matrices/vecotor after the coordinate transform to the CoM
        void computeTransformation(const Eigen::VectorXd &Vel_);
        void computeJacDotQDot();
        void computeJdqdCOMlinear();
        void update(Eigen::Matrix4d &eigenWorld_H_base, Eigen::Matrix<double,12,1> &eigenJointPos, Eigen::Matrix<double,12,1> &eigenJointVel, Eigen::Matrix<double,6,1> &eigenBasevel, Eigen::Vector3d &eigenGravity);			
        
        Eigen::Vector3d get_eebr_pos();
        Eigen::Vector3d get_eebl_pos();
        Eigen::Vector3d get_eefl_pos();
        Eigen::Vector3d get_eefr_pos();
        Eigen::Vector3d get_eebr_vel();
        Eigen::Vector3d get_eebl_vel();
        Eigen::Vector3d get_eefl_vel();
        Eigen::Vector3d get_eefr_vel();

        Eigen::VectorXd get_wcom();
        Eigen::VectorXd qpproblem_stance( Eigen::VectorXd &Wcom_des);
        Eigen::VectorXd qpproblem_swing( Eigen::VectorXd &Wcom_des, bool *check_contact);

        void set_initial_pose();

        // starts the controller threaded loop
        void start();

        std::thread ctrl_thread;

        Eigen::Matrix<double,6,1>* getBasePosPtr();

    private:
        
        // Helper variable to add a 2*PI offset to the dog's yaw
        int yaw_counter;

        ros::NodeHandle _nh;
        ros::Subscriber _gazebo_clock_sub; 
        ros::Subscriber _joint_state_sub; 
        ros::Subscriber _model_state_sub; 

        // ros::Subscriber _navigation_sub;

        // ros::Subscriber _eebl_sub;
        // ros::Subscriber _eebr_sub;
        // ros::Subscriber _eefl_sub;
        // ros::Subscriber _eefr_sub;
        
        ros::Publisher  _joint_pub;
        
        ros::Publisher _leg_plot_pub;
        ros::Publisher _leg_ref_plot_pub;
        ros::Publisher _com_plot_pub;
        ros::Publisher _com_ref_plot_pub;

        Eigen::Matrix4d _world_H_base;
        Eigen::Matrix<double,12,1> _jnt_pos; 
        Eigen::Matrix<double,12,1> _jnt_vel;
        Eigen::Matrix<double,6,1> _base_pos;
        Eigen::Matrix<double,6,1> _base_vel;

        string _model_name;

        bool _first_wpose;
        bool _first_jpos;
        unordered_map<int, string> _id2idname;  
        unordered_map<int, int> _id2index;      
        unordered_map<int, int> _index2id;      
        Eigen::VectorXd x_eigen;
       

        bool _contact_br;
        bool _contact_bl;
        bool _contact_fl;
        bool _contact_fr;


        // int for DoFs number
        unsigned int n;
        // Total mass of the robot
        double robot_mass;
        // KinDynComputations element
        iDynTree::KinDynComputations kinDynComp;
        // world to floating base transformation
        iDynTree::Transform world_H_base;
        // Joint position
        iDynTree::VectorDynSize jointPos;
        // Floating base velocity
        iDynTree::Twist         baseVel;
        // Joint velocity
        iDynTree::VectorDynSize jointVel;
        // Gravity acceleration
        iDynTree::Vector3       gravity; 
        // Position vector base+joints
        iDynTree::VectorDynSize  qb;
        // Velocity vector base+joints
        iDynTree::VectorDynSize  dqb;
        // Position vector COM+joints
        iDynTree::VectorDynSize  q;
        // Velocity vector COM+joints
        iDynTree::VectorDynSize  dq;
        // Joints limit vector
        iDynTree::VectorDynSize  qmin;
        iDynTree::VectorDynSize  qmax;
        // Center of Mass Position
        iDynTree::Vector6 CoM;
        // Center of mass velocity
        iDynTree::Vector6 CoM_vel;
        //Mass matrix
        iDynTree::FreeFloatingMassMatrix MassMatrix;
        //Bias Matrix
        iDynTree::VectorDynSize Bias;
        //Gravity Matrix
        iDynTree::MatrixDynSize GravMatrix;
        // Jacobian
        iDynTree::MatrixDynSize Jac;
        // Jacobian derivative
        iDynTree::MatrixDynSize JacDot;
        //CoM Jacobian
        iDynTree::MatrixDynSize Jcom;
        // Bias acceleration J_dot*q_dot
        iDynTree::MatrixDynSize Jdqd;
        // Transformation Matrix
        iDynTree::MatrixDynSize T;
        // Transformation matrix time derivative
        iDynTree::MatrixDynSize T_inv_dot;
        //Model
        iDynTree::Model model;
        iDynTree::ModelLoader mdlLoader;
        //Mass matrix in CoM representation
        iDynTree::FreeFloatingMassMatrix MassMatrixCOM;
        //Bias Matrix in CoM representation
        iDynTree::VectorDynSize BiasCOM;
        //Gravity Matrix in CoM representation
        iDynTree::MatrixDynSize GravMatrixCOM;
        // Jacobian in CoM representation
        iDynTree::MatrixDynSize JacCOM;
        //Jacobian in CoM representation (only linear part)
        iDynTree::MatrixDynSize JacCOM_lin;
        // Bias acceleration J_dot*q_dot in CoM representation
        iDynTree::MatrixDynSize JdqdCOM;
        // Bias acceleration J_dot*q_dot in CoM representation
        iDynTree::MatrixDynSize JdqdCOM_lin;

        // current swing leg & com references 
        Eigen::Matrix<double,6,1> x_sw_des, x_sw_dot_des, x_sw_ddot_des;
        Eigen::Matrix<double,6,1> rc_ref, rc_dot_ref, rc_ddot_ref;

        double t; //per prendere il riferimento attuale dalla spline towr
        ros::Time start_time; //tempo iniziale dell'ottimizzazione per spline
        bool my_check_contact[4]; // da caricare in ordine BR BL FL FR
        Eigen::Matrix<double,6,1> destination; // received by nav (continuously updated)
        Eigen::Matrix<double,6,1> current_goal; // fixed goal. updated everytime we replan via towr
        Eigen::Matrix<double,3,3> rot;

        bool mutex; // mutex to avoid accessing destination while it's being updated
        

};

#endif