#include "ros/ros.h"
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include <tf/transform_listener.h>
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/Twist.h"  
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector> 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class HelperNode {

	public:
		HelperNode();
		void modelStateCallback(const gazebo_msgs::ModelStates & msg);
        Eigen::Matrix<double,6,1> _base_pos;
        Eigen::Matrix<double,6,1> _base_vel;
	
	private:
		ros::NodeHandle _nh;
		ros::Subscriber _topic_sub;
		ros::Subscriber _model_state_sub;
};


HelperNode::HelperNode() {
    _model_state_sub = _nh.subscribe("/gazebo/model_states", 0, &HelperNode::modelStateCallback, this);
}

void HelperNode::modelStateCallback(const gazebo_msgs::ModelStates & msg) {

    bool found = false;
    int index = 0;
	std::string _model_name = "dogbot";
    while( !found  && index < msg.name.size() ) {

        if( msg.name[index] == _model_name )
            found = true;
        else index++;
    }

    if( found ) {
        
        //quaternion
        tf::Quaternion q(msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z,  msg.pose[index].orientation.w);
        q.normalize();
        Eigen::Matrix<double,3,3> rot;
        tf::matrixTFToEigen(tf::Matrix3x3(q),rot);

        //Roll, pitch, yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        //Set base pos (position and orientation)
        _base_pos << msg.pose[index].position.x, msg.pose[index].position.y, msg.pose[index].position.z, roll, pitch, yaw;

        //Set base vel
        _base_vel << msg.twist[index].linear.x, msg.twist[index].linear.y, msg.twist[index].linear.z, msg.twist[index].angular.x, msg.twist[index].angular.y, msg.twist[index].angular.z;
    }
}


int main( int argc, char** argv ) {

	ros::init(argc, argv, "converter");
	HelperNode rs;
	ros::NodeHandle nh;
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("base_link_gt", 50);
	ros::spinOnce();
    tf::TransformBroadcaster odom_broadcaster;

	ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
	double actual_angle;
	double actual_angle_vel;
	Eigen::Matrix<double,2,1> actual_pose;
	Eigen::Matrix<double,2,1> actual_vel;

	ros::Rate rate(25); //Rate

	actual_pose = rs._base_pos.block(0,0,2,1);
	actual_angle = rs._base_pos(5);
	sleep(1);
    
    while (ros::ok()) {
                
		ros::spinOnce();
		current_time = ros::Time::now();
		actual_pose = rs._base_pos.block(0,0,2,1);
		actual_vel = rs._base_vel.block(0,0,2,1);
		actual_angle = rs._base_pos(5);
		actual_angle_vel = rs._base_vel(5);
		//since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(actual_angle);
   
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "base_link_gt";
   
        odom_trans.transform.translation.x = actual_pose(0);
        odom_trans.transform.translation.y = actual_pose(1);
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;
   
		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "map";

		//set the position
		odom.pose.pose.position.x = actual_pose(0);
		odom.pose.pose.position.y = actual_pose(1);
		odom.pose.pose.position.z = 0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link_gt";
		odom.twist.twist.linear.x = actual_vel(0);
		odom.twist.twist.linear.y = actual_vel(1);
		odom.twist.twist.angular.z = actual_angle_vel;

		//publish the message
		odom_pub.publish(odom);

		last_time = current_time;
        
		
		rate.sleep();
		
	}

}