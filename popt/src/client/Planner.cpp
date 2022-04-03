
#include "Planner.h"


Planner::Planner(ros::NodeHandle& nh)
{	
	// set the node
	_nh = nh;

	// clean data
	data = "";

	// default target id
	target_id = "id_2";

	// default goals
	Eigen::Vector2d temp(0,0);
	goals.push_back(temp);

	counter = 0;

	home << 0,0;

	measurements = false;

	// ======== Register callbacks ========

	map_sub = _nh.subscribe("/map", 1, &Planner::mapCallback, this);
    // model_state_sub = _nh.subscribe("/gazebo/model_states", 0, &Planner::modelStateCallback, this);
	img_sub = _nh.subscribe("/dogbot/rrbot/camera1/image_raw", 1, &Planner::imageCallback, this);
	// nav_pub = _nh.advertise<geometry_msgs::Twist>("/navigation", 1);

}

Planner::~Planner()
{
	master_thread.join();
}

void Planner::master()
{	
	bool found = false; // true if we found the target QR code

	ros::Duration(1.0).sleep(); // wait for a while (controller keeps robot upright in the meantime)

    while (ros::ok())
	{
		// pop next goal from goal vector
		current_goal = goals.back();
		goals.pop_back();

		// go to next goal
		artpot(); 
		// look around, try to find the qr code
		found = findQR();

		// now we're ready to head to next room (or home)

		// if there are no goals left, or target found --> go home
		if(goals.empty() || found) {
			std::cout << "GOING HOME! \n";
			current_goal = home;
			artpot(); // go home
			break; // exit while loop -> end navigation
		}
	}

}

bool Planner::findQR()
{

	data = ""; 
	geometry_msgs::Twist msg;

	double start_yaw = (*_base_pos)(5);

	ros::Rate rate(20);
	while(ros::ok())
	{
		// std::cout << data << "\n";
		// std::cout << target_id << "\n";

		// found right qr 
		if (data == target_id) {std::cout << "QR CODE FOUND! GOING HOME...\n"; return true;}

		// found a qr, but its not the right one --> return false
		else if (data != "" && data != target_id) {std::cout << "WRONG QR CODE! GOING TO NEXT ROOM... \n"; return false;}
	
		// there's no qr -> return false
		else if (data == "" && ((*_base_pos)(5) - start_yaw) > 1.7*M_PI) {std::cout << "NO QR CODE FOUND! GOING TO NEXT ROOM... \n";return false;}

		// not yet, try again
		else {
			msg.linear.x = (*_base_pos)(0);
			msg.linear.y = (*_base_pos)(1);
			msg.linear.z = (*_base_pos)(2);
			msg.angular.x = 0;
			msg.angular.y = 0;
       	 	msg.angular.z = (*_base_pos)(5) + 0.15;
			// nav_pub.publish(msg);
			if (navCallback_)
			{
				std::thread publish(navCallback_,msg);
				publish.detach();
			}
		}
		rate.sleep();
	}
	return false;
}

void Planner::artpot() {

	// nav goal
	geometry_msgs::Twist msg; 

	// A force gain
	const unsigned int Ka = 40;

	// R force gain
	const unsigned int Kr = 60; // 50

	// max gait
	// 0.7 quicker
	// 0.5 safer
	const double step_size = 0.05;

	// obstacle influence radius
	const double obst_treshold = 1.0;

	// rate
	ros::Rate rate(25);

	// seed rand
	srand (static_cast <unsigned> (time(0)));

	while (ros::ok())
	{	

		Eigen::Vector2d current_pos = (*_base_pos).head<2>();
		double current_yaw = (*_base_pos)(5);

		Eigen::Vector2d error_pos;
		error_pos = current_goal - current_pos;
		double error_pos_norm = (error_pos.norm());

		if(error_pos_norm < 0.2) break; // if super close to goal

		// =========== Attractive force ===========

		// conical potential when far from goal (constant force)
		// parabolic potential when close (linear force)
		Eigen::Vector2d A_force(0,0);
		if(error_pos_norm > 1){ // when far
            A_force = Ka * (error_pos / error_pos_norm); 
		}
		else { // when close
			A_force = Ka * error_pos;
		}

		// =========== Repulsive force ===========

		Eigen::Vector2d R_force(0,0);

		Eigen::Vector2d dist(0,0);
		double dist_norm = 0;
		Eigen::Vector2d min_dist(0,0);
		double min_dist_norm = 100;
		unsigned int cells = 0;

		// iterate over occupied grid cells
		for(int i=0; i<(X_Y_Obstacles.size()); i+=2){
			
			dist(0) = current_pos(0)-X_Y_Obstacles[i];
			dist(1) = current_pos(1)-X_Y_Obstacles[i+1];
			dist_norm = (dist.norm());
			
			// if cell inside treshold -> set min distance
			if(dist_norm < obst_treshold && dist_norm < min_dist_norm){
				min_dist = dist;
				min_dist_norm = dist_norm;
			}
		}

		if (min_dist_norm < obst_treshold)
		R_force = Kr*min_dist*((1/min_dist_norm)-(1/obst_treshold))/(pow(min_dist_norm,2));

		// =========== Total force ===========

		Eigen::Vector2d force = A_force + R_force;

		// =========== Force too low -> random waypoint ===========

		// generate random force (range 0 to 40). then waypoint will be assigned accordingly
		if(force.norm() < 2) { // empiric threshold 
			std::cout << "FORCE TOO LOW -> RANDOM WAYPOINT!\n";
			force(1) = static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(40.0)));
			force(2) = static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(40.0)));
		}

		// =========== Compute position waypoint ===========
		
		// truncate delta to match gait requirements
		Eigen::Vector2d delta;
		if ((force.norm()) > step_size) delta = force / (force.norm()) * step_size;
		else delta = force;
		
		Eigen::Vector2d waypoint;
		waypoint = current_pos + delta;

		// =========== Compute yaw reference ===========

		// ref is 0.1 rad in the direction of the goal
		double goal_yaw = atan2(A_force(1), A_force(0)) + 1.57;
		double error_yaw = goal_yaw - current_yaw;
		double ref_yaw = current_yaw + 0.1 * error_yaw / abs(error_yaw);

		// =========== Load msg & publish nav goal ===========

		// if big yaw error --> just rotate
		// else go forward too
		if (abs(error_yaw) > 0.05 )
		{
			msg.linear.x = current_pos(0);
			msg.linear.y = current_pos(1);
			msg.angular.z = ref_yaw;
		}
		else 
		{
			msg.linear.x = waypoint(0);
			msg.linear.y = waypoint(1);
			msg.angular.z = goal_yaw;
		}

		// =========== Some couts ===========

		// std::cout << " ------------- \n";
		// std::cout << "current_pos \n" << current_pos << "\n";
		// std::cout << "waypoint \n" << waypoint << "\n";
		// std::cout << "delta \n" << (delta.norm()) << "\n";
		// std::cout << "min dist norm \n" << min_dist_norm << "\n";
		// std::cout << "ref_yaw \n" << ref_yaw << "\n";
		// std::cout << " Total Force " << (force.norm()) << "\n";
		
		// nav_pub.publish(msg);

		if (navCallback_)
		{
			std::thread publish(navCallback_,msg);
			publish.detach();
		}

		rate.sleep();
	}

}

void Planner::setGoals(const std::vector<Eigen::Vector2d> & goals_)
{

	goals = goals_;

}

void Planner::setHome(const Eigen::Vector2d & home_)
{
	home = home_;
}

void Planner::setCode(const std::string & code_)
{
	target_id = code_;
}

void Planner::start()
{

	master_thread = std::thread(&Planner::master,this);
	// master_thread = std::thread(&Planner::TESTROT,this);

}

void Planner::goHome()
{
	// TODO 
}

void Planner::stop()
{
	// TODO
}

void Planner::mapCallback(nav_msgs::OccupancyGridConstPtr occupancy_grid)
{
	for (int width=0; width < occupancy_grid->info.width; ++width)
    {
        for (int height=0; height < occupancy_grid->info.height; ++height)
        {
            if(occupancy_grid->data[height*occupancy_grid->info.width + width] > 0)
            {
               float x = width * occupancy_grid->info.resolution + occupancy_grid->info.resolution / 2 + occupancy_grid->info.origin.position.x;
               float y = height * occupancy_grid->info.resolution + occupancy_grid->info.resolution / 2 + occupancy_grid->info.origin.position.y;
			   X_Y_Obstacles.push_back(x);
			   X_Y_Obstacles.push_back(y);
            }
        }
    }
}

// void Planner::modelStateCallback(const gazebo_msgs::ModelStates & msg)
// {
// 	bool found = false;
//     int index = 0;
// 	std::string _model_name = "dogbot";
//     while( !found  && index < msg.name.size() ) {

//         if( msg.name[index] == _model_name )
//             found = true;
//         else index++;
//     }

//     if( found ) {
        
//         // get quaternion from gazebo
//         tf::Quaternion q(msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z,  msg.pose[index].orientation.w);
//         q.normalize();
//         Eigen::Matrix<double,3,3> rot;
//         tf::matrixTFToEigen(tf::Matrix3x3(q),rot);

//         // convert to rpy
//         double roll, pitch, yaw;
// 		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
// 		// std::cout << "old_yaw: " << old_yaw << "\n";
// 		// std::cout << "yaw: " << yaw << "\n\n";

// 		// check if there's a jump in the gazebo yaw
// 		if(abs(yaw - old_yaw) > M_PI){
//         	if(yaw < old_yaw){ //crossing counterclockwise
//             	counter++; 
// 				std::cout << "\n(PLANNER)    CROSSING COUNTERCLOCKWISE: YAW += 2PI\n";
//         	}
//         	else if(yaw > old_yaw){ //crossing clockwise
//             	counter--;
// 				std::cout << "\n(PLANNER)    CROSSING CLOCKWISE: YAW -= 2PI\n";
//         	}
//     	}

// 		old_yaw = yaw; // save old yaw
// 		yaw += counter*2*M_PI; // the real yaw

//         //Set base pos with updated yaw
//         (*_base_pos) << msg.pose[index].position.x, msg.pose[index].position.y, msg.pose[index].position.z, roll, pitch, yaw;

// 		// std::cout << "\n(PLANNER)    YAW: " << (*_base_pos)(5) << "\n";

//         //Set base vel
//         _base_vel << msg.twist[index].linear.x, msg.twist[index].linear.y, msg.twist[index].linear.z, msg.twist[index].angular.x, msg.twist[index].angular.y, msg.twist[index].angular.z;
		
// 		measurements = true;

//     }
// }

void Planner::setBasePos(Eigen::Matrix<double,6,1>* ptr)
{
	_base_pos = ptr;
}

void Planner::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
	cv_bridge::CvImageConstPtr cv_image;

	try {
		cv_image = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	detect(cv_image->image);
}

void Planner::detect(const cv::Mat& image)
{

	zbar::ImageScanner scanner;
	scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1); //Config settings

	cv::Mat grayImg;
	cv::cvtColor(image, grayImg, CV_BGR2GRAY);


	const auto width = image.cols;
	const auto height = image.rows;

	zbar::Image img(width, height, "Y800", grayImg.data, width * height);
	scanner.scan(img);


	for(zbar::Image::SymbolIterator symbol = img.symbol_begin(); symbol!= img.symbol_end(); ++symbol)
	{
		//std::cout<< symbol->get_data()<<std::endl;
		data = symbol->get_data();
	}
}

void Planner::TESTROT()
{
	// the robot oscillates around M_PI

	geometry_msgs::Twist msg;

	ros::Rate rate(20);

	double start_yaw = (*_base_pos)(5);

	bool flag = true;

	while(ros::ok())
	{

		if (flag) { // clockwise
			msg.linear.x = (*_base_pos)(0);
			msg.linear.y = (*_base_pos)(1);
			msg.linear.z = (*_base_pos)(2);
			msg.angular.x = 0;
			msg.angular.y = 0;
       	 	msg.angular.z = (*_base_pos)(5) + 0.15;
			if (navCallback_)
			{
				std::thread publish(navCallback_,msg);
				publish.detach();
			}
			if (((*_base_pos)(5)-start_yaw) > 1.2*M_PI) flag = false;
		}

		else if(!flag) { // rotate counterclockwise 
			msg.linear.x = (*_base_pos)(0);
			msg.linear.y = (*_base_pos)(1);
			msg.linear.z = (*_base_pos)(2);
			msg.angular.x = 0;
			msg.angular.y = 0;
       	 	msg.angular.z = (*_base_pos)(5) - 0.15;
			if (navCallback_)
			{
				std::thread publish(navCallback_,msg);
				publish.detach();
			}
			if (((*_base_pos)(5)-start_yaw) < 0.8*M_PI) flag = true;
		}

		rate.sleep();
	}

}
