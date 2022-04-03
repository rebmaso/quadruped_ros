#ifndef INCLUDE_PLANNER_HPP_
#define INCLUDE_PLANNER_HPP_


#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"  
#include "nav_msgs/OccupancyGrid.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include "tf_conversions/tf_eigen.h"
// #include <iDynTree/Core/EigenHelpers.h>

#include <vector>
#include <cmath>  
#include <cstdlib>
#include <thread>
#include <math.h>
#include <time.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "zbar.h"

#include <iostream>
#include <tf/tf.h>
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"


// this class is tasked with going to predefined locations and looking for a target qr code

typedef std::function<void(const geometry_msgs::Twist &)> navCallback;

class Planner
{
private:

	// coordinates to reach (in opposite order)
	std::vector<Eigen::Vector2d> goals;
    
	// where we are going right now
	Eigen::Vector2d current_goal;

	// home coordinates
	Eigen::Vector2d home;

	// target qr code
	std::string target_id;

	// detected qr code
	std::string data;

	// occupied grid locations
	std::vector<double> X_Y_Obstacles; 

	// current pose (points to pose in controller object)
	Eigen::Matrix<double,6,1>* _base_pos;

	// is set by modelstate callback when meas are available -> starts processing
	bool measurements;

	// to fix yaw bug
	int counter; // add counter*2pi to yaw meas
	double old_yaw; // old raw measurement to detect jump in yaw

	// to send goals to the controller
	navCallback navCallback_;


	// ======== Ros stuff ========

	ros::NodeHandle _nh;

	// listens to map topic from gmapping
	ros::Subscriber map_sub;

	// listens to current base pose
	ros::Subscriber model_state_sub;

	// listens to camera topic to find qr
	ros::Subscriber img_sub;

	// sends nav messages to control node
	ros::Publisher nav_pub;

	// ======== Some private funcs ========
    
	// is in charge of using the artpot algorithm (threaded)
	void master();
	std::thread master_thread;
	
	// the artificial potential fields algorithm
	void artpot();

	// look around to find the qr code
	bool findQR();

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // to avoid alignment issues, just in case

	Planner(ros::NodeHandle& nh);

	Planner() = delete;

	~Planner();

	void setGoals(const std::vector<Eigen::Vector2d> & goals_);

	void setHome(const Eigen::Vector2d & home);

	void setCode(const std::string & code_);

	// launches master thread
	void start();
    
	// forces the robot to head home
	void goHome();

	// forces the robot to stop in place
	void stop();

	void mapCallback(nav_msgs::OccupancyGridConstPtr occupancy_grid);

	void modelStateCallback(const gazebo_msgs::ModelStates & msg);

	void imageCallback(const sensor_msgs::ImageConstPtr &image);

	void detect(const cv::Mat& image);

	void TESTROT();

	void setNavCallback(navCallback navCallback) {
		navCallback_ = navCallback;
	}

	void setBasePos(Eigen::Matrix<double,6,1>* ptr);
};

#endif