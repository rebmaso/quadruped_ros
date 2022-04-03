#include "Controller.h"
#include "Planner.h"

int main(int argc, char** argv) {
    
    ros::init( argc, argv, "dogbot_control");
    ros::NodeHandle nh;

    // =========== Low-level control ===========
    
    // Set the controller
    Controller dc(nh);
    // Waits for sensor measurements
    dc.start();

    // =========== Navigation ===========

    // Set the planner
    Planner pl(nh);

    // Set base pos reference from the controller
    pl.setBasePos(dc.getBasePosPtr());

    // Register controller nav callback (better to avoid ros for internal comms)
    pl.setNavCallback(std::bind(&Controller::navCallback, &dc, std::placeholders::_1));

    // // 2d goals (test)
	// Eigen::Vector2d goal_1(0,-5);
	// Eigen::Vector2d goal_2(0,-2);
	// Eigen::Vector2d goal_3(1,-3);

    // // 2d goals (small room)
	// Eigen::Vector2d goal_1;
	// Eigen::Vector2d goal_2;
	// Eigen::Vector2d goal_3;
    // goal_1 << 0.21, -7,95;
    // goal_2 << 3.86, -1.00;
    // goal_3 << 3.86, -8.00;

    // 2d goals (big room)
	Eigen::Vector2d goal_1;
	Eigen::Vector2d goal_2;
	Eigen::Vector2d goal_3;
    goal_1 << -6.76, -4.71;
    goal_2 << 6.5, -2.36;
    goal_3 << -1.35, 4.7;

	std::vector<Eigen::Vector2d> goals; // we load this with the 2d goal coords
    
	// the vector will be: head --> 3 , 2 , 1 <-- back
	goals.push_back(goal_3);
	goals.push_back(goal_2);
	goals.push_back(goal_1);
    
    Eigen::Vector2d home(0,0);

	// target QR code
    std::string code;

    if (argc == 2)
    {
        code = argv[1];
    }
    else code = "id_1"; // default


    pl.setCode(code);
    pl.setGoals(goals);
    pl.setHome(home);
    // Waits for sensor measurements
    pl.start();

    // =========== Gazebo ===========

    // Starts gazebo simulation
    ros::ServiceClient unpauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty unpauseSrv;
    unpauseGazebo.call(unpauseSrv);
    // Triggers callbacks. Processing starts
    ros::spin();
}