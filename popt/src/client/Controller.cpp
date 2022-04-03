
#include "Controller.h"


Controller::Controller(ros::NodeHandle& nh) {

    _nh = nh;

    set_initial_pose();
   
    // _gazebo_clock_sub = _nh.subscribe("/rosgraph_msgs/Clock", 0, &Controller::gazeboclockCallback, this);
    _joint_state_sub = _nh.subscribe("/dogbot/joint_states", 0, &Controller::jointStateCallback, this);
    _model_state_sub = _nh.subscribe("/gazebo/model_states", 0, &Controller::modelStateCallback, this);

    // _navigation_sub = _nh.subscribe("/navigation", 1, &Controller::navCallback, this);

    // _eebl_sub = _nh.subscribe("/dogbot/back_left_contactsensor_state",0, &Controller::eebl_cb, this);
    // _eefl_sub = _nh.subscribe("/dogbot/front_left_contactsensor_state",0, &Controller::eefl_cb, this);
    // _eebr_sub = _nh.subscribe("/dogbot/back_right_contactsensor_state",0, &Controller::eebr_cb, this);
    // _eefr_sub = _nh.subscribe("/dogbot/front_right_contactsensor_state",0,&Controller::eefr_cb, this);
    
    _joint_pub = _nh.advertise<std_msgs::Float64MultiArray>("/dogbot/joint_position_controller/command", 1);

    // _leg_plot_pub = _nh.advertise<geometry_msgs::Vector3>("/aaa/leg_plot", 1);
    // _leg_ref_plot_pub = _nh.advertise<geometry_msgs::Vector3>("/aaa/leg_ref_plot", 1);

    // _com_plot_pub = _nh.advertise<geometry_msgs::Vector3>("/aaa/com_plot", 1);
    // _com_ref_plot_pub = _nh.advertise<geometry_msgs::Vector3>("/aaa/com_ref_plot", 1);

    _model_name = "dogbot";

    yaw_counter = 0;

    mutex = true;

    _first_wpose = false;
    _first_jpos = false;
    _contact_br = true; 
    _contact_bl = true; 
    _contact_bl = true; 
    _contact_fr = true;


    std::string path = ros::package::getPath("dogbot_description");
    path += "/urdf/dogbot.urdf";

    createrobot(path);

    model = kinDynComp.model();
	kinDynComp.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);
	// Resize matrices of the class given the number of DOFs
    n = model.getNrOfDOFs();
    
    robot_mass = model.getTotalMass();
    jointPos = iDynTree::VectorDynSize(n);
    baseVel = iDynTree::Twist();
    jointVel = iDynTree::VectorDynSize(n);
	q = iDynTree::VectorDynSize(6+n);
	dq = iDynTree::VectorDynSize(6+n);
	qb = iDynTree::VectorDynSize(6+n);
	dqb=iDynTree::VectorDynSize(6+n);
	qmin= iDynTree::VectorDynSize(n);
	qmax= iDynTree::VectorDynSize(n);
	Bias=iDynTree::VectorDynSize(n+6);
	GravMatrix=iDynTree::MatrixDynSize(n+6,1);
    MassMatrix=iDynTree::FreeFloatingMassMatrix(model) ;
    Jcom=iDynTree::MatrixDynSize(3,6+n);
	Jac=iDynTree::MatrixDynSize(24,6+n);	
	JacDot=iDynTree::MatrixDynSize(24,6+n);
	Jdqd=iDynTree::MatrixDynSize(24,1);
    T=iDynTree::MatrixDynSize(6+n,6+n);
	T_inv_dot=iDynTree::MatrixDynSize(6+n,6+n);
    MassMatrixCOM=iDynTree::FreeFloatingMassMatrix(model) ;
    BiasCOM=iDynTree::VectorDynSize(n+6);
	GravMatrixCOM=iDynTree::MatrixDynSize(n+6,1);
	JacCOM=iDynTree::MatrixDynSize(24,6+n);
	JacCOM_lin=iDynTree::MatrixDynSize(12,6+n);
	JdqdCOM=iDynTree::MatrixDynSize(24,1);
	JdqdCOM_lin=iDynTree::MatrixDynSize(12,1);
	x_eigen= Eigen::VectorXd::Zero(30);
}

Controller::~Controller() {
    ctrl_thread.join();
}

void Controller::set_initial_pose(){

  //Ros Client
  ros::ServiceClient set_model_configuration_srv = _nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
  ros::ServiceClient set_model_state_srv = _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  // Start the robot in position (stand up) 
  gazebo_msgs::SetModelConfiguration robot_init_config;
  robot_init_config.request.model_name = "dogbot";
  robot_init_config.request.urdf_param_name = "robot_description";
  robot_init_config.request.joint_names.push_back("back_left_roll_joint");
  robot_init_config.request.joint_names.push_back("back_left_pitch_joint");
  robot_init_config.request.joint_names.push_back("back_left_knee_joint");
  robot_init_config.request.joint_names.push_back("back_right_roll_joint");
  robot_init_config.request.joint_names.push_back("back_right_pitch_joint");
  robot_init_config.request.joint_names.push_back("back_right_knee_joint");
  robot_init_config.request.joint_names.push_back("front_left_roll_joint");
  robot_init_config.request.joint_names.push_back("front_left_pitch_joint");
  robot_init_config.request.joint_names.push_back("front_left_knee_joint");
  robot_init_config.request.joint_names.push_back("front_right_roll_joint");
  robot_init_config.request.joint_names.push_back("front_right_pitch_joint");
  robot_init_config.request.joint_names.push_back("front_right_knee_joint");
  robot_init_config.request.joint_positions.push_back( 0.0004875394147498824);
  robot_init_config.request.joint_positions.push_back( -0.884249947977489);
  robot_init_config.request.joint_positions.push_back(-1.6039026405138666);
  robot_init_config.request.joint_positions.push_back( 0.0006243098169198547);
  robot_init_config.request.joint_positions.push_back(0.8861978063639038);
  robot_init_config.request.joint_positions.push_back(1.6032646991719783);
  robot_init_config.request.joint_positions.push_back(-3.197670677312914e-05);
  robot_init_config.request.joint_positions.push_back(-0.8848124990461947);
  robot_init_config.request.joint_positions.push_back(-1.6039627256817717);
  robot_init_config.request.joint_positions.push_back(-0.0005127385581351618);
  robot_init_config.request.joint_positions.push_back(0.886353788084274);
  robot_init_config.request.joint_positions.push_back( 1.60361055049274);
  
  if(set_model_configuration_srv.call(robot_init_config))
    ROS_INFO("Robot configuration set.");
  else
    ROS_INFO("Failed to set robot configuration.");
  
  gazebo_msgs::SetModelState robot_init_state;
  robot_init_state.request.model_state.model_name = "dogbot";
  robot_init_state.request.model_state.reference_frame = "world";
  robot_init_state.request.model_state.pose.position.x=-0.00;
  robot_init_state.request.model_state.pose.position.y= -0.034102251365;
  robot_init_state.request.model_state.pose.position.z=0.45059040502;
  robot_init_state.request.model_state.pose.orientation.x=0.0;
  robot_init_state.request.model_state.pose.orientation.y=0.0;
  robot_init_state.request.model_state.pose.orientation.z=0.0;
  robot_init_state.request.model_state.pose.orientation.w=1; 
  
  if(set_model_state_srv.call(robot_init_state))
    ROS_INFO("Robot state set.");
  else
    ROS_INFO("Failed to set robot state.");
}

void Controller::createrobot(std::string modelFile) {  
    
    if( !mdlLoader.loadModelFromFile(modelFile) ) {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        return ;
    }
    if( !kinDynComp.loadRobotModel(mdlLoader.model()) )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return ;
    }

    _id2idname.insert( pair< int, string > ( 0, kinDynComp.getDescriptionOfDegreeOfFreedom(0) ));
    _id2idname.insert( pair< int, string > ( 1, kinDynComp.getDescriptionOfDegreeOfFreedom(1) ));
    _id2idname.insert( pair< int, string > ( 2, kinDynComp.getDescriptionOfDegreeOfFreedom(2) ));
    _id2idname.insert( pair< int, string > ( 3, kinDynComp.getDescriptionOfDegreeOfFreedom(3) ));
    _id2idname.insert( pair< int, string > ( 4, kinDynComp.getDescriptionOfDegreeOfFreedom(4) ));
    _id2idname.insert( pair< int, string > ( 5, kinDynComp.getDescriptionOfDegreeOfFreedom(5) ));
    _id2idname.insert( pair< int, string > ( 6, kinDynComp.getDescriptionOfDegreeOfFreedom(6) ));
    _id2idname.insert( pair< int, string > ( 7, kinDynComp.getDescriptionOfDegreeOfFreedom(7) ));
    _id2idname.insert( pair< int, string > ( 8, kinDynComp.getDescriptionOfDegreeOfFreedom(8) ));
    _id2idname.insert( pair< int, string > ( 9, kinDynComp.getDescriptionOfDegreeOfFreedom(9) ));
    _id2idname.insert( pair< int, string > ( 10, kinDynComp.getDescriptionOfDegreeOfFreedom(10) ));
    _id2idname.insert( pair< int, string > ( 11, kinDynComp.getDescriptionOfDegreeOfFreedom(11) ));
}

// Compute matrix transformation T needed to recompute matrices/vector after the coordinate transform to the CoM
void Controller::computeTransformation(const Eigen::VectorXd &Vel_) {

    //Set ausiliary matrices
    iDynTree::MatrixDynSize Jb(6,6+n);
    iDynTree::MatrixDynSize Jbc(3,n);
    iDynTree::Vector3 xbc;
    iDynTree::MatrixDynSize xbc_hat(3,3);
    iDynTree::MatrixDynSize xbc_hat_dot(3,3);
    iDynTree::MatrixDynSize Jbc_dot(6,6+n);
    iDynTree::Vector3 xbo_dot;

    //Set ausiliary matrices
    iDynTree::Vector3 xbc_dot;

    // Compute T matrix
    // Get jacobians of the floating base and of the com
    kinDynComp.getFrameFreeFloatingJacobian(0,Jb);
    kinDynComp.getCenterOfMassJacobian(Jcom);

    // Compute jacobian Jbc=d(xc-xb)/dq used in matrix T
    toEigen(Jbc)<<toEigen(Jcom).block<3,12>(0,6)-toEigen(Jb).block<3,12>(0,6);

    // Get xb (floating base position) and xc ( com position)
    iDynTree::Position xb = world_H_base.getPosition();
    iDynTree::Position xc= kinDynComp.getCenterOfMassPosition();

    // Vector xcb=xc-xb
    toEigen(xbc)=toEigen(xc)-toEigen(xb);

    // Skew of xcb
    toEigen(xbc_hat)<<0, -toEigen(xbc)[2], toEigen(xbc)[1],
    toEigen(xbc)[2], 0, -toEigen(xbc)[0],                          
    -toEigen(xbc)[1], toEigen(xbc)[0], 0;

    Eigen::Matrix<double,6,6> X;
    X<<Eigen::MatrixXd::Identity(3,3), toEigen(xbc_hat).transpose(), 
    Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3);

    Eigen::MatrixXd Mb_Mj= toEigen(MassMatrix).block(0,0,6,6).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(toEigen(MassMatrix).block(0,6,6,12));
    Eigen::Matrix<double,6,12> Js=X*Mb_Mj;

    // Matrix T for the transformation
    toEigen(T)<<Eigen::MatrixXd::Identity(3,3), toEigen(xbc_hat).transpose(), Js.block(0,0,3,12),
    Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3), Js.block(3,0,3,12),
    Eigen::MatrixXd::Zero(12,3),  Eigen::MatrixXd::Zero(12,3), Eigen::MatrixXd::Identity(12,12);

    //Compute time derivative of T 
    // Compute derivative of xbc
    toEigen(xbc_dot)=toEigen(kinDynComp.getCenterOfMassVelocity())-toEigen(baseVel.getLinearVec3());
    Eigen::VectorXd  mdr=robot_mass*toEigen(xbc_dot);
    Eigen::Matrix<double,3,3> mdr_hat;
    mdr_hat<<0, -mdr[2], mdr[1],
    mdr[2], 0, -mdr[0],                          
    -mdr[1], mdr[0], 0;

    //Compute skew of xbc
    toEigen(xbc_hat_dot)<<0, -toEigen(xbc_dot)[2], toEigen(xbc_dot)[1],
    toEigen(xbc_dot)[2], 0, -toEigen(xbc_dot)[0],                          
    -toEigen(xbc_dot)[1], toEigen(xbc_dot)[0], 0;

    Eigen::Matrix<double,6,6> dX;
    dX<<Eigen::MatrixXd::Zero(3,3), toEigen(xbc_hat_dot).transpose(),
    Eigen::MatrixXd::Zero(3,6);
    // Time derivative of Jbc
    kinDynComp.getCentroidalAverageVelocityJacobian(Jbc_dot);

    Eigen::Matrix<double,6,6> dMb;
    dMb<<Eigen::MatrixXd::Zero(3,3), mdr_hat.transpose(),
    mdr_hat, Eigen::MatrixXd::Zero(3,3);

    Eigen::MatrixXd inv_dMb1=(toEigen(MassMatrix).block(0,0,6,6).transpose().bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dMb.transpose())).transpose();
    Eigen::MatrixXd inv_dMb2=-(toEigen(MassMatrix).block(0,0,6,6).bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve( inv_dMb1));

    Eigen::Matrix<double,6,12> dJs=dX*Mb_Mj+X*inv_dMb2*toEigen(MassMatrix).block(0,6,6,12);

    toEigen(T_inv_dot)<<Eigen::MatrixXd::Zero(3,3), toEigen(xbc_hat_dot), -dJs.block(0,0,3,12),
    Eigen::MatrixXd::Zero(15,18);

}

// Compute Jacobian
void  Controller::computeJac() {     

    //Set ausiliary matrices
    iDynTree::MatrixDynSize Jac1(6,6+n);
    iDynTree::MatrixDynSize Jac2(6,6+n);
    iDynTree::MatrixDynSize Jac3(6,6+n);
    iDynTree::MatrixDynSize Jac4(6,6+n);

    // Compute Jacobian for each leg

    // Jacobian for back right leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("back_left_foot"), Jac1); //"back_right_foot"

    // Jacobian for back left leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("back_right_foot"),Jac2);  //"back_left_foot"

    // Jacobian for front left leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("front_left_foot"), Jac3);

    // Jacobian for front right leg
    kinDynComp.getFrameFreeFloatingJacobian( kinDynComp.getFrameIndex("front_right_foot"), Jac4);

    // Full Jacobian
    toEigen(Jac)<<toEigen(Jac1), toEigen(Jac2), toEigen(Jac3), toEigen(Jac4);
    
}

void Controller::ComputeJaclinear() {
    
  Eigen::Matrix<double,12,24> B;
  B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);

  toEigen(JacCOM_lin)=B*toEigen(JacCOM);
    
}

void Controller::computeJdqdCOMlinear()
{
	Eigen::Matrix<double,12,24> B;
    B<< Eigen::MatrixXd::Identity(3,3) , Eigen::MatrixXd::Zero(3,21),
      Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,15),
	  Eigen::MatrixXd::Zero(3,12), Eigen::MatrixXd::Identity(3,3),  Eigen::MatrixXd::Zero(3,9),
	  Eigen::MatrixXd::Zero(3,18), Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,3);


    toEigen(JdqdCOM_lin)= Eigen::MatrixXd::Zero(12,1);
    toEigen(JdqdCOM_lin)=B*toEigen(JdqdCOM);
	
}

// Compute Bias acceleration: J_dot*q_dot
void  Controller::computeJacDotQDot() {
    

    // Bias acceleration for back right leg
    iDynTree::Vector6 Jdqd1=kinDynComp.getFrameBiasAcc("back_left_foot"); //"back_right_foot"

    // Bias acceleration for back left leg
    iDynTree::Vector6 Jdqd2=kinDynComp.getFrameBiasAcc("back_right_foot"); //"back_left_foot"

    // Bias acceleration for front left leg
    iDynTree::Vector6 Jdqd3=kinDynComp.getFrameBiasAcc("front_left_foot");

    // Bias acceleration for front right leg
    iDynTree::Vector6 Jdqd4=kinDynComp.getFrameBiasAcc("front_right_foot"); 
    toEigen(Jdqd)<<toEigen(Jdqd1), toEigen(Jdqd2), toEigen(Jdqd3), toEigen(Jdqd4);

	
}

Eigen::Vector3d Controller::get_eebr_pos(){
	iDynTree::Transform  World_br;
    World_br=kinDynComp.getWorldTransform("back_right_foot"); //"back_right_foot"
	return toEigen(World_br.getPosition());
}

Eigen::Vector3d Controller::get_eebl_pos(){
	iDynTree::Transform  World_bl;
    World_bl=kinDynComp.getWorldTransform("back_left_foot"); //"back_left_foot"
	return toEigen(World_bl.getPosition());
}

Eigen::Vector3d Controller::get_eefl_pos(){
	iDynTree::Transform  World_fl;
    World_fl=kinDynComp.getWorldTransform("front_left_foot");
	return toEigen(World_fl.getPosition());
}

Eigen::Vector3d Controller::get_eefr_pos(){
	iDynTree::Transform  World_fr;
    World_fr=kinDynComp.getWorldTransform("front_right_foot");
	return toEigen(World_fr.getPosition());
}

Eigen::Vector3d Controller::get_eebr_vel() // BR
{   iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel("back_right_foot");  //"back_right_foot"

	return toEigen(br_vel.getLinearVec3() );

}

Eigen::Vector3d Controller::get_eebl_vel() //BL
{
	iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel("back_left_foot");  //"back_left_foot"

	return toEigen(br_vel.getLinearVec3() );

}

Eigen::Vector3d Controller::get_eefl_vel() //FL
{
iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel("front_left_foot");

	return toEigen(br_vel.getLinearVec3() );

}

Eigen::Vector3d Controller::get_eefr_vel() //FR
{
	iDynTree::Twist br_vel;
	br_vel=kinDynComp.getFrameVel("front_right_foot");

	return toEigen(br_vel.getLinearVec3() );

}

Eigen::VectorXd Controller::get_wcom(){

    Eigen::MatrixXd Kp_com; 
    Eigen::MatrixXd Kd_com;
    Eigen::VectorXd gravity_aug(6);
    gravity_aug << 0,0,9.8,0,0,0;

    Kp_com = 3500*1*Eigen::MatrixXd::Identity(6,6); //tune
    Kd_com = 50*1*Eigen::MatrixXd::Identity(6,6); // 2.5 to damp oscillations

    Eigen::VectorXd deltax(6);
    deltax << rc_ref.segment(0,3) - toEigen(CoM).segment(0,3), toEigen(world_H_base.getRotation())*(rc_ref.segment(3,3) - toEigen(CoM).segment(3,3));

    Eigen::VectorXd deltav(6);
    deltav << rc_dot_ref.segment(0,3) - toEigen(CoM_vel).segment(0,3), rc_dot_ref.segment(3,3) - toEigen(CoM_vel).segment(3,3);

    Eigen::VectorXd wcom_des(6);

    //wcom_des = Kp_com*(rc_ref - toEigen(CoM)) + Kd_com*(rc_dot_ref - toEigen(CoM_vel)) + robot_mass*gravity_aug + toEigen(MassMatrixCOM).block(0,0,6,6)*rc_ddot_ref;
    wcom_des = Kp_com*deltax + Kd_com*deltav + robot_mass*gravity_aug + toEigen(MassMatrixCOM).block(0,0,6,6)*rc_ddot_ref;

    return wcom_des;

}

Eigen::VectorXd Controller::qpproblem_stance( Eigen::VectorXd &Wcom_des)
{
	// Set variables
   int variables=30;
   alglib::real_2d_array Q, R, L;
   alglib::real_1d_array c, bl, bu;
   alglib::integer_1d_array Lt;
   
   Q.setlength(variables,variables);
   c.setlength(variables);
   bl.setlength(variables);
   bu.setlength(variables);
   L.setlength(86,31);
   Lt.setlength(86);


   // Taking Jacobian for CoM and joints
   Eigen::Matrix<double, 12, 6> Jstcom= toEigen(JacCOM_lin).block(0,0,12,6);
   Eigen::Matrix<double, 12, 12> Jstj= toEigen(JacCOM_lin).block(0,6,12,12);

    Eigen::Matrix<double, 12, 18> Jst= toEigen(JacCOM_lin);

   // cost function quadratic matrix
   Eigen::Matrix<double,12,30> Sigma= Eigen::Matrix<double,12,30>::Zero();
   Sigma.block(0,18,12,12)= Eigen::Matrix<double,12,12>::Identity();
   
   Eigen::Matrix<double,6,30>  T_s= Jstcom.transpose()*Sigma;
   Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();

   Eigen::Matrix<double,30,30> eigenQ2= T_s.transpose()*eigenQ1*T_s;
   Eigen::Matrix<double,30,30> eigenR= Eigen::Matrix<double,30,30>::Identity();
   Eigen::Matrix<double,30,30> eigenQ= eigenQ2+Eigen::Matrix<double,30,30>::Identity();
 

	 
   for ( int i = 0; i < eigenQ.rows(); i++ ){
        for ( int j = 0; j < eigenQ.cols(); j++ )
             Q(i,j) = eigenQ(i,j);
    } 

    // cost function linear matrix
	Eigen::Matrix<double,30,1> eigenc= -T_s.transpose()*eigenQ1.transpose()*Wcom_des; 

   for ( int i = 0; i < eigenc.rows(); i++ ){
       for ( int j = 0; j < eigenc.cols(); j++ )
             c(i) = eigenc(i,j);
    }

    alglib::minqpstate state;
	// Create QP optimizer
    alglib::minqpcreate(30,state);
	alglib::minqpsetquadraticterm( state,Q);
    alglib::minqpsetlinearterm(state,c);
	
	//Equality constraints
	Eigen::Matrix<double,18, 30> eigenA= Eigen::Matrix<double,18,30>::Zero();
	
	eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);

	eigenA.block(0,18,6,12)=-Jstcom.transpose();

    eigenA.block(6,0,12,6)=Jstcom;

    eigenA.block(6,6,12,12)=Jstj;

    // Known term
    Eigen::Matrix<double,18, 1> eigenb= Eigen::Matrix<double,18,1>::Zero();

    Eigen::Matrix<double,1,6> grav;
	grav<<0,0,9.8,0,0,0;
	eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1); //+Jstcom.transpose()*fext;

	eigenb.block(6,0,12,1)=-toEigen(JdqdCOM_lin);
   
    Eigen::Matrix<double,18,18> P=Eigen::Matrix<double,18,18>::Identity()-toEigen(JacCOM_lin).transpose()*(toEigen(JacCOM_lin)*toEigen(MassMatrixCOM).inverse()*toEigen(JacCOM_lin).transpose()).inverse()*toEigen(JacCOM_lin)*toEigen(MassMatrixCOM).inverse();

	//Inequality Constraints

	Eigen::Matrix<double,68, 30> eigenD= Eigen::Matrix<double,68,30>::Zero();
	
	 // Torque limits
	eigenD.block(20,6,12,12)=toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(20,18,12,12)=-Jstj.transpose();

	eigenD.block(32,6,12,12)=-toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(32,18,12,12)=Jstj.transpose();

	eigenD.block(44,6,12,12)=Eigen::Matrix<double,12,12>::Identity();

    eigenD.block(56,6,12,12)=-Eigen::Matrix<double,12,12>::Identity();
    
	//Friction
	   double mu=0.4; // tune
	   Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
	   n<< 0,
	       0,
		   1;

	   Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
	   t1<< 1,
	       0,
		   0;

       Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
	   t2<<0,
	       1,
		   0;

	   Eigen::Matrix<double,5,3> cfr=Eigen::Matrix<double,5,3>::Zero();
  
	   cfr<<(-mu*n+t1).transpose(),
	        (-mu*n+t2).transpose(),
			-(mu*n+t1).transpose(),
			-(mu*n+t2).transpose(),
			-n.transpose();
     
	    Eigen::Matrix<double,20,12> Dfr=Eigen::Matrix<double,20,12>::Zero();

		for(int i=0; i<4; i++)
		{
			Dfr.block(0+5*i,0+3*i,5,3)=cfr;
		}
		

    eigenD.block(0,18,20,12)=Dfr;


    // Known terms for inequality
	Eigen::Matrix<double,68, 1> eigenC= Eigen::Matrix<double,68,1>::Zero();
	
	// Torque limits
    Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
	Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones();

    Eigen::Matrix<double,12, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,12,1);
    for(int i=0; i<4; i++)
	{
		eigenC.block(4+i*5,0,1,1)<<-20;
	}
	eigenC.block(20,0,12,1)=tau_max-eigenBiascom;
	eigenC.block(32,0,12,1)=-(tau_min-eigenBiascom);
    
      // Joints limits
     double deltat=0.01;
     Eigen::Matrix<double,12, 1> eigenq=toEigen(q).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigendq=toEigen(dq).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigenqmin=toEigen(qmin);
	 Eigen::Matrix<double,12, 1> eigenqmax=toEigen(qmax);
	 Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
	 Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

     eigenC.block(44,0,12,1)=ddqmax;
	 eigenC.block(56,0,12,1)=-ddqmin;
	

	 Eigen::Matrix<double,18,18> Si;
	  Si<<Eigen::Matrix<double,6,18>::Zero(),
	      Eigen::Matrix<double,12,6>::Zero(),Eigen::Matrix<double,12,12>::Identity();
    

	iDynTree::MatrixDynSize Jac1(6,18);
	iDynTree::MatrixDynSize Jac2(6,18);
    kinDynComp.getFrameFreeFloatingJacobian(7,Jac2);
	toEigen(Jac1)=toEigen(Jac2)*toEigen(T).inverse();

	
     //Linear constraints matrix
    Eigen::Matrix<double,86, 31> eigenL= Eigen::Matrix<double,86,31>::Zero();

	eigenL<< eigenA,eigenb,
	         eigenD, eigenC;

    
   
    for ( int i = 0; i < eigenL.rows(); i++ ){
		 if (i < 18)
            {
				Lt(i) = 0.0; 
			}
        else
           {
            Lt(i) = -1.0; 
		   }
		   for ( int j = 0; j < eigenL.cols(); j++ )
             L(i,j) = eigenL(i,j);
	}

    
    // Set qp
    alglib::minqpsetlc(state, L, Lt);
	alglib::minqpsetscaleautodiag(state);
	alglib::real_1d_array x_;
    
    alglib::minqpreport rep;
	//alglib::minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 15);
    alglib::minqpsetalgodenseaul(state, 1.0e-2, 1.0e+4, 5);


    alglib::minqpoptimize(state);

	// Solve qp
    alglib::minqpresults(state, x_, rep);

	for ( int j = 0; j < x_eigen.size(); j++ )
             x_eigen(j)=x_(j);
    
	Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);
	tau=toEigen(MassMatrixCOM).block(6,6,12,12)*x_eigen.block(6,0,12,1)-Jstj.transpose()*x_eigen.block(18,0,12,1) +eigenBiascom;
	
	return tau;

}

Eigen::VectorXd Controller::qpproblem_swing( Eigen::VectorXd &Wcom_des, bool *check_contact)
{
    Eigen::Matrix<double,12,1> fext = Eigen::MatrixXd::Zero(12,1);

	int swl1, swl2, stl1, stl2;

  if(!check_contact[0] && !check_contact[2]){ // swing: BR e FL
    swl1 = 3;//BR
    swl2 = 6;//FL TOWR INIZIA CON QUEST
    stl1 = 0;
    stl2 = 9;
  }
  else if(!check_contact[1] && !check_contact[3]){ // swing: BL e FR
    swl1 = 0; //BL
    swl2 = 9;//FR
    stl1 = 3;
    stl2 = 6;
  }
  else{ std::cout << "errore in qpproblem swing: gambe strane"; }

	// Set variables
   int variables=30;
   alglib::real_2d_array Q, R, L;
   alglib::real_1d_array c, bl, bu;
   alglib::integer_1d_array Lt;
   
   Q.setlength(variables,variables);
   c.setlength(variables);
   bl.setlength(variables);
   bu.setlength(variables);
   L.setlength(82,31);
   Lt.setlength(82);


   // Taking Jacobian for CoM and joints
   Eigen::Matrix<double, 6, 6> Jstcom= Eigen::Matrix<double,6,6>::Zero();
    Jstcom.block(0,0,3,6)= toEigen(JacCOM_lin).block(stl1,0,3,6);
	Jstcom.block(3,0,3,6)= toEigen(JacCOM_lin).block(stl2,0,3,6);

   Eigen::Matrix<double, 6, 12> Jstj= Eigen::Matrix<double,6,12>::Zero();
    Jstj.block(0,0,3,12)=toEigen(JacCOM_lin).block(stl1,6,3,12);
    Jstj.block(3,0,3,12)=toEigen(JacCOM_lin).block(stl2,6,3,12);

  Eigen::Matrix<double, 6, 18> Jst= Eigen::Matrix<double,6,18>::Zero();
 Jst.block(0,0,3,18)=toEigen(JacCOM_lin).block(stl1,0,3,18);
    Jst.block(3,0,3,18)=toEigen(JacCOM_lin).block(stl2,0,3,18);

   Eigen::Matrix<double, 6, 6> Jswcom= Eigen::Matrix<double,6,6>::Zero();
    Jswcom.block(0,0,3,6)= toEigen(JacCOM_lin).block(swl1,0,3,6);
	 Jswcom.block(3,0,3,6)= toEigen(JacCOM_lin).block(swl2,0,3,6);

   Eigen::Matrix<double, 6, 12> Jswj=  Eigen::Matrix<double,6,12>::Zero();
   Jswj.block(0,0,3,12)=toEigen(JacCOM_lin).block(swl1,6,3,12);
   Jswj.block(3,0,3,12)=toEigen(JacCOM_lin).block(swl2,6,3,12);

   // cost function quadratic matrix
   Eigen::Matrix<double,6,30> Sigma= Eigen::Matrix<double,6,30>::Zero();
   Sigma.block(0,18,6,6)= Eigen::Matrix<double,6,6>::Identity();
   Eigen::Matrix<double,6,30>  T_s= Jstcom.transpose()*Sigma;

   Eigen::Matrix<double,6,6> eigenQ1= 50*Eigen::Matrix<double,6,6>::Identity();
   Eigen::Matrix<double,30,30> eigenQ2= T_s.transpose()*eigenQ1*T_s;
   Eigen::Matrix<double,30,30> eigenR= Eigen::Matrix<double,30,30>::Identity();
   eigenR.block(24,24,6,6)=1000*Eigen::Matrix<double,6,6>::Identity();
   
   Eigen::Matrix<double,30,30> eigenQ= eigenQ2+eigenR;
 	 
   for ( int i = 0; i < eigenQ.rows(); i++ ){
        for ( int j = 0; j < eigenQ.cols(); j++ )
             Q(i,j) = eigenQ(i,j);
    } 

    // cost function linear matrix
	Eigen::Matrix<double,30,1> eigenc= -T_s.transpose()*eigenQ1.transpose()*Wcom_des; 

   for ( int i = 0; i < eigenc.rows(); i++ ){
       for ( int j = 0; j < eigenc.cols(); j++ )
             c(i) = eigenc(i,j);
    }

    alglib::minqpstate state;
	// Create QP optimizer
    alglib::minqpcreate(30,state);
	alglib::minqpsetquadraticterm( state,Q);
    alglib::minqpsetlinearterm(state,c);	

	//Equality constraints
	Eigen::Matrix<double,12, 30> eigenA= Eigen::Matrix<double,12,30>::Zero();
	
	eigenA.block(0,0,6,6)=toEigen(MassMatrixCOM).block(0,0,6,6);

	eigenA.block(0,18,6,6)=-Jstcom.transpose();

    eigenA.block(6,0,6,6)=Jstcom;

    eigenA.block(6,6,6,12)=Jstj;

    // Known term
    Eigen::Matrix<double,12, 1> eigenb= Eigen::Matrix<double,12,1>::Zero();

    Eigen::Matrix<double,6,1> Jdqdst= Eigen::Matrix<double,6,1>::Zero();
	 Jdqdst<<toEigen(JdqdCOM_lin).block(stl1,0,3,1),
	         toEigen(JdqdCOM_lin).block(stl2,0,3,1);
			 
    Eigen::Matrix<double,1,6> grav;
	grav<<0,0,9.8,0,0,0;

       Eigen::Matrix<double,6,1> fext_st;
   fext_st.block(0,0,3,1)=fext.block(stl1,0,3,1);
   fext_st.block(3,0,3,1)=fext.block(stl2,0,3,1);
	eigenb.block(0,0,6,1)=-toEigen(BiasCOM).block(0,0,6,1)+Jstcom.transpose()*fext_st;

	eigenb.block(6,0,6,1)=-Jdqdst;
    
    Eigen::Matrix<double,18,18> P=Eigen::Matrix<double,18,18>::Identity()-Jst.transpose()*(Jst*toEigen(MassMatrixCOM).inverse()*Jst.transpose()).inverse()*Jst*toEigen(MassMatrixCOM).inverse();
     
	//Inequality Constraints

	Eigen::Matrix<double,70,30> eigenD= Eigen::Matrix<double,70,30>::Zero();
	
	 // Torque limits
	eigenD.block(10,6,12,12)=toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(10,18,12,6)=-Jstj.transpose();

	eigenD.block(22,6,12,12)=-toEigen(MassMatrixCOM).block(6,6,12,12);

    eigenD.block(22,18,12,6)=Jstj.transpose();
    
    eigenD.block(34,0,3,6)=Jswcom.block(0,0,3,6);

    eigenD.block(34,6,3,12)=Jswj.block(0,0,3,12);

	eigenD.block(37,0,3,6)=Jswcom.block(3,0,3,6);

    eigenD.block(37,6,3,12)=Jswj.block(3,0,3,12);

	eigenD.block(34,24,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(37,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(40,0,3,6)=-Jswcom.block(0,0,3,6);

    eigenD.block(40,6,3,12)=-Jswj.block(0,0,3,12);

	eigenD.block(43,0,3,6)=-Jswcom.block(3,0,3,6);

    eigenD.block(43,6,3,12)=-Jswj.block(3,0,3,12);

    eigenD.block(40,24,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(43,27,3,3)=-Eigen::Matrix<double,3,3>::Identity();

	eigenD.block(46,6,12,12)=Eigen::Matrix<double,12,12>::Identity();

    eigenD.block(58,6,12,12)=-Eigen::Matrix<double,12,12>::Identity();
    
	//Friction
	   double mu=0.4;
	   Eigen::Matrix<double,3, 1> n= Eigen::Matrix<double,3,1>::Zero();
	   n<< 0,
	       0,
		   1;

	   Eigen::Matrix<double,3, 1> t1= Eigen::Matrix<double,3,1>::Zero();
	   t1<< 1,
	       0,
		   0;

       Eigen::Matrix<double,3, 1> t2= Eigen::Matrix<double,3,1>::Zero();
	   t2<<0,
	       1,
		   0;

	   Eigen::Matrix<double,5,3> cfr=Eigen::Matrix<double,5,3>::Zero();
  
	   cfr<<(-mu*n+t1).transpose(),
	        (-mu*n+t2).transpose(),
			-(mu*n+t1).transpose(),
			-(mu*n+t2).transpose(),
			-n.transpose();
     
	    Eigen::Matrix<double,10,6> Dfr=Eigen::Matrix<double,10,6>::Zero();

		for(int i=0; i<2; i++)
		{
			Dfr.block(0+5*i,0+3*i,5,3)=cfr;
		}
		

    eigenD.block(0,18,10,6)=Dfr;

    // Known terms for inequality
	Eigen::Matrix<double,70, 1> eigenC= Eigen::Matrix<double,70,1>::Zero();
	
	// Torque limits
    Eigen::Matrix<double,12,1> tau_max=60*Eigen::Matrix<double,12,1>::Ones();
	Eigen::Matrix<double,12,1> tau_min=-60*Eigen::Matrix<double,12,1>::Ones();

    Eigen::Matrix<double,12, 1> eigenBiascom=toEigen(BiasCOM).block(6,0,12,1);  
	eigenC.block(10,0,12,1)=tau_max-eigenBiascom;
	eigenC.block(22,0,12,1)=-(tau_min-eigenBiascom);
    
      // Joints limits
     double deltat=0.01;
     Eigen::Matrix<double,12, 1> eigenq=toEigen(q).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigendq=toEigen(dq).block(6,0,12,1);
	 Eigen::Matrix<double,12, 1> eigenqmin=toEigen(qmin);
	 Eigen::Matrix<double,12, 1> eigenqmax=toEigen(qmax);
	 Eigen::Matrix<double,12, 1> ddqmin=(2/pow(deltat,2))*(eigenqmin-eigenq-deltat*eigendq);
	 Eigen::Matrix<double,12, 1> ddqmax=(2/pow(deltat,2))*(eigenqmax-eigenq-deltat*eigendq);

     eigenC.block(46,0,12,1)=ddqmax;
	 eigenC.block(58,0,12,1)=-ddqmin;

	 Eigen::Matrix<double,6,1> Jdqdsw= Eigen::Matrix<double,6,1>::Zero();
	 Jdqdsw<<toEigen(JdqdCOM_lin).block(swl1,0,3,1),
	         toEigen(JdqdCOM_lin).block(swl2,0,3,1);
 
	  Eigen::Matrix<double,6,1> fext_lambda= Eigen::Matrix<double,6,1>::Zero();

	  Eigen::Matrix<double,18,18> Si;
	  Si<<Eigen::Matrix<double,6,18>::Zero(),
	      Eigen::Matrix<double,12,6>::Zero(),Eigen::Matrix<double,12,12>::Identity();
    
	fext_lambda<<toEigen(JacCOM_lin).block(swl1,0,3,18)*toEigen(MassMatrixCOM).inverse()*Si.transpose()*P*toEigen(JacCOM_lin).transpose()*fext,
	            toEigen(JacCOM_lin).block(swl2,0,3,18)*toEigen(MassMatrixCOM).inverse()*Si.transpose()*P*toEigen(JacCOM_lin).transpose()*fext;


    Eigen::MatrixXd Kp = 500*1*Eigen::MatrixXd::Identity(6,6); // Tune this
    Eigen::MatrixXd Kd =50*1.7*Eigen::MatrixXd::Identity(6,6);

    Eigen::Matrix<double,6,1> e_sw ;
    Eigen::Matrix<double,6,1> d_e_sw ;

    if(!check_contact[0] && !check_contact[2]){
        e_sw << x_sw_des.block(0,0,3,1)- get_eebr_pos(), x_sw_des.block(3,0,3,1)- get_eefl_pos();
        d_e_sw << x_sw_dot_des.block(0,0,3,1)- get_eebr_vel(), x_sw_dot_des.block(3,0,3,1)- get_eefl_vel();}

    else {
        e_sw << x_sw_des.block(0,0,3,1)- get_eebl_pos(), x_sw_des.block(3,0,3,1)- get_eefr_pos();
        d_e_sw << x_sw_dot_des.block(0,0,3,1)- get_eebl_vel(), x_sw_dot_des.block(3,0,3,1)- get_eefr_vel();
    }

    Eigen::Matrix<double,6,1> dd_xsw_cmd = x_sw_ddot_des+ Kd*d_e_sw + Kp*e_sw;
    

    Eigen::Matrix<double,12,1> Jdot_qd_lin =  toEigen(JdqdCOM_lin);
    Eigen::Matrix<double,6,1> Jswdot_qd_lin= Eigen::Matrix<double,6,1>::Zero();

    Jswdot_qd_lin << Jdot_qd_lin.block(swl1,0,3,1),Jdot_qd_lin.block(swl2,0,3,1);

    eigenC.block(34,0,6,1)= dd_xsw_cmd-Jdqdsw-fext_lambda;
    eigenC.block(40,0,6,1)= -dd_xsw_cmd+Jdqdsw+fext_lambda;

     //Linear constraints matrix
    Eigen::Matrix<double,82, 31> eigenL= Eigen::Matrix<double,82,31>::Zero();


    eigenL<< eigenA,eigenb,
                eigenD, eigenC;

    for ( int i = 0; i < eigenL.rows(); i++ ){
            if (i < 12)
            {
                Lt(i) = 0.0; 
            }
        else
            {
            Lt(i) = -1.0; 
            }
            for ( int j = 0; j < eigenL.cols(); j++ )
                L(i,j) = eigenL(i,j);
    }

    // Set qp
    alglib::minqpsetlc(state, L, Lt);
    alglib::minqpsetscaleautodiag(state);
    alglib::real_1d_array x_;

    alglib::minqpreport rep;
    //alglib::minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 15);
    alglib::minqpsetalgodenseaul(state, 1.0e-2, 1.0e+4, 5);


    alglib::minqpoptimize(state);

    // Solve qp
    alglib::minqpresults(state, x_, rep);


    for ( int j = 0; j < x_eigen.size(); j++ )
                x_eigen(j)=x_(j);

    Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);
    tau=toEigen(MassMatrixCOM).block(6,6,12,12)*x_eigen.block(6,0,12,1)-Jstj.transpose()*x_eigen.block(18,0,6,1) +eigenBiascom;

    return tau;
}

// Get joints position and velocity
void Controller::jointStateCallback(const sensor_msgs::JointState & msg) {

    if( _first_jpos == false ) {

        for( int i=0; i<12; i++) {
            bool found = false;
            int index = 0;
            while( !found && index <  msg.name.size() ) {
                if( msg.name[index] == _id2idname.at( i )    ) {
                    found = true;

                    _id2index.insert( pair< int, int > ( i, index ));
                    _index2id.insert( pair< int, int > ( index, i ));

                }
                else index++;
            }
        }
    }

    for( int i=0; i<12; i++ ) {
        _jnt_pos( i, 0) = msg.position[    _id2index.at(i)    ];
    }

    for( int i=0; i<12; i++ ) {
        _jnt_vel( i, 0) = msg.velocity[    _id2index.at(i)    ];
    }
    
    _first_jpos = true;
}

// Get base position and velocity
void Controller::modelStateCallback(const gazebo_msgs::ModelStates & msg) {

    bool found = false;
    int index = 0;
    while( !found  && index < msg.name.size() ) {

        if( msg.name[index] == _model_name )
            found = true;
        else index++;
    }

    if( found ) {
        
        _world_H_base.setIdentity();
        
        //quaternion
        tf::Quaternion q(msg.pose[index].orientation.x, msg.pose[index].orientation.y, msg.pose[index].orientation.z,  msg.pose[index].orientation.w);
        q.normalize();
        tf::matrixTFToEigen(tf::Matrix3x3(q),rot);

        //Roll, pitch, yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        //Set base pos (position and orientation)
        _base_pos << msg.pose[index].position.x, msg.pose[index].position.y, msg.pose[index].position.z, roll, pitch, yaw;
        //Set transformation matrix
        _world_H_base.block(0,0,3,3)= rot;
        _world_H_base.block(0,3,3,1)= _base_pos.block(0,0,3,1);

        //Set base vel
        _base_vel << msg.twist[index].linear.x, msg.twist[index].linear.y, msg.twist[index].linear.z, msg.twist[index].angular.x, msg.twist[index].angular.y, msg.twist[index].angular.z;
        _first_wpose = true;
    }
}

void Controller::navCallback(const geometry_msgs::Twist& msg){ //callback per msg navigazione

  mutex = false;

  destination(0) = msg.linear.x; destination(1) = msg.linear.y; destination(2) = msg.linear.z; //parte lineare x,y,z
  destination(3) = msg.angular.x; destination(4) = msg.angular.y; destination(5) = msg.angular.z; //parte angolare

  mutex = true;
   
}

//Update elements of the class given the new state

void Controller::update (Eigen::Matrix4d &eigenWorld_H_base, Eigen::Matrix<double,12,1> &eigenJointPos, Eigen::Matrix<double,12,1> &eigenJointVel, Eigen::Matrix<double,6,1> &eigenBasevel, Eigen::Vector3d &eigenGravity)
{   

   
    // Update joints, base and gravity from inputs
    iDynTree::fromEigen(world_H_base,eigenWorld_H_base);
    iDynTree::toEigen(jointPos) = eigenJointPos;
    iDynTree::fromEigen(baseVel,eigenBasevel);
    toEigen(jointVel) = eigenJointVel;
    toEigen(gravity)  = eigenGravity;

    // useless variable, delete later
    Eigen::Vector3d worldeigen=toEigen(world_H_base.getPosition());
    while (worldeigen==Eigen::Vector3d::Zero()){
        iDynTree::fromEigen(world_H_base,eigenWorld_H_base);
        worldeigen=toEigen(world_H_base.getPosition());
    }

    // use this temp variable to check if odometry yaw is ok
    iDynTree::Vector3 base_angle;
    base_angle=world_H_base.getRotation().asRPY();
    double yaw = base_angle(2);
    yaw += yaw_counter*2*M_PI;

    // check if we need to update yaw_counter & yaw
    if( abs(current_goal(5)-yaw) > M_PI)
    {
        if((current_goal(5)- yaw) > M_PI){ //crossing pi counterclockwise
            yaw_counter++; 
            std::cout << "\n(CONTROLLER) CROSSING COUNTERCLOCKWISE: YAW += 2PI\n";
        }
        else if((current_goal(5)- yaw) < -M_PI){ //crossing pi clockwise
            yaw_counter--; 
            std::cout << "\n(CONTROLLER) CROSSING CLOCKWISE: YAW -= 2PI\n";
        }
        yaw = yaw + yaw_counter*2*M_PI; 
    }

    base_angle(2) = yaw;

    // std::cout << "\n(CONTROLLER) YAW: " << base_angle(2) << "\n";

    // integrate the updated yaw into the tf
    iDynTree::Rotation fixed_rot = iDynTree::Rotation::RPY(base_angle(0),base_angle(1),base_angle(2));
    world_H_base.setRotation(fixed_rot);

    // Only now that we have corrected the yaw, can we update kindyncomp
    // and compute all class member variables!

    //Set the state for the robot 
    kinDynComp.setRobotState(world_H_base,jointPos,
    baseVel,jointVel,gravity);

    // Compute position of the center of mass
    toEigen(CoM)<<toEigen(kinDynComp.getCenterOfMassPosition()), toEigen(base_angle);
    
	// Compute velocity of the center of mass
    toEigen(CoM_vel)<<toEigen(kinDynComp.getCenterOfMassVelocity()), eigenBasevel.block(3,0,3,1);
		   
    // Compute position base +joints
	toEigen(qb)<<toEigen(world_H_base.getPosition()), toEigen(base_angle), eigenJointPos;
    // Compute position COM+joints
	toEigen(q)<<toEigen(CoM), eigenJointPos;
   	toEigen(dq)<<toEigen(CoM_vel), eigenJointVel;
	toEigen(dqb) << eigenBasevel, eigenJointVel;
   
	// Joint limits
    toEigen(qmin)<< -1.75 , -1.75,-1.75,-1.75,-1.58, -2.62, -3.15, -0.02,  -1.58, -2.62, -3.15, -0.02;
    toEigen(qmax)<< 1.75, 1.75, 1.75, 1.75, 3.15, 0.02, 1.58, 2.62,  3.15, 0.02, 1.58, 2.62;

    // Get mass, bias (C(q,v)*v+g(q)) and gravity (g(q)) matrices
    //Initialize ausiliary vector
    iDynTree::FreeFloatingGeneralizedTorques bias_force(model);
    iDynTree::FreeFloatingGeneralizedTorques grav_force(model);
    //Compute Mass Matrix
    kinDynComp.getFreeFloatingMassMatrix(MassMatrix); 
    //Compute Coriolis + gravitational terms (Bias)
    kinDynComp.generalizedBiasForces(bias_force);
    toEigen(Bias)<<iDynTree::toEigen(bias_force.baseWrench()),
        iDynTree::toEigen(bias_force.jointTorques());

    
    //Compute Gravitational term
    kinDynComp.generalizedGravityForces(grav_force);
    toEigen(GravMatrix)<<iDynTree::toEigen(grav_force.baseWrench()),
            iDynTree::toEigen(grav_force.jointTorques());

    computeJac();	
    // Compute Bias Acceleration -> J_dot*q_dot
    computeJacDotQDot();
    
    Eigen::Matrix<double, 18,1> q_dot;

    q_dot<< eigenBasevel,
            eigenJointVel;

    // Compute Matrix needed for transformation from floating base representation to CoM representation
    computeTransformation(q_dot);
    // Compute Mass Matrix in CoM representation 
    toEigen(MassMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T).inverse();
    // Compute Coriolis+gravitational term in CoM representation
    toEigen(BiasCOM)=toEigen(T).transpose().inverse()*toEigen(Bias)+toEigen(T).transpose().inverse()*toEigen(MassMatrix)*toEigen(T_inv_dot)*toEigen(dq);

    // Compute gravitational term in CoM representation	
    toEigen(GravMatrixCOM)=toEigen(T).transpose().inverse()*toEigen(GravMatrix);
    // Compute Jacobian term in CoM representation
    toEigen(JacCOM)=toEigen(Jac)*toEigen(T).inverse();
    ComputeJaclinear();
    // Compute Bias Acceleration -> J_dot*q_dot  in CoM representation
    toEigen(JdqdCOM)=toEigen(Jdqd)+toEigen(Jac)*toEigen(T_inv_dot)*toEigen(dq);
    computeJdqdCOMlinear();	
}

void  Controller::publish_cmd(  Eigen::VectorXd tau  ) {
    std_msgs::Float64MultiArray tau1_msg;
    
    // Fill Command message
    for(int i=11; i>=0; i--) {
        tau1_msg.data.push_back(  tau( _index2id.at(i) )    );
    }

    //Sending command
    _joint_pub.publish(tau1_msg);

}

void Controller::eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr){
	if(eebr->states.empty()){ 
        _contact_br= false;
	}
	else {
		_contact_br= true;
	}
}

void Controller::eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl){

	if(eefl->states.empty()){ 
        _contact_fl= false;
	}
	else {
		_contact_fl= true;
    }
}

void Controller::eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl){

	if(eebl->states.empty()){ 
        _contact_bl= false;
	}
	else {
	    _contact_bl= true;
    }
}

void Controller::eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr){
	if(eefr->states.empty()){ 
        _contact_fr= false;
	}
	else {
		_contact_fr= true;
	}
}
 
void Controller::ctrl_loop() {

    ros::ServiceClient pauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    ros::ServiceClient unpauseGazebo = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty pauseSrv;
    std_srvs::Empty unpauseSrv;

    // declare params
    int gait_flag = 2; // C1
    float duration = 0.75; // (towr) tune this value 0.55
    int nst = 0;
    Eigen::VectorXd wcom_des(6);
    Eigen::VectorXd tau_star(12);
    Eigen::VectorXd rc_star;
    Eigen::VectorXd qdd_star;
    Eigen::VectorXd fgr_star;
    towr::SplineHolder solution; // towr output spline
    //towr::NlpFormulation formulation_; // defined later
    bool contact = false;
    int array[4] = {1,0,2,3}; // to access towr swing legs ref in the right order
    int temp = 0;
    bool flag = false; 
    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.8;

 while(!_first_wpose || !_first_jpos ){ // wait until good meas are read
     // unpauseGazebo.call(unpauseSrv);
     ROS_INFO_STREAM_ONCE("Controller waiting on sensor measurements...");
     //ros::spinOnce();
  }

    // update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);
    ros::Rate rate(1000);

    while( ros::ok() ) {

        // ros::spinOnce();

        // update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);

        pauseGazebo.call(pauseSrv);
     
       if (!flag){ // compute new towr trajectory
            towr::NlpFormulation formulation_;
            update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);
            while(!mutex){} // wait until destination is done being updated
            current_goal = destination;
            get_trajectory(toEigen(CoM), toEigen(CoM_vel), current_goal, get_eebl_pos(), get_eebr_pos(), get_eefl_pos(), get_eefr_pos(), gait_flag, duration, solution, formulation_);
            flag = true;
            start_time = ros::Time::now();
        }

        //while((ros::Time::now() - start_time).toSec()< formulation_.params_.ee_phase_durations_.at(1)[0]){
        //pauseGazebo.call(pauseSrv);

        t = (ros::Time::now() - start_time).toSec(); // current t, to extract the right towr reference

        // when the refs reach the final points of the spline (all legs in stance): 
        // keep constant ref for 0.05 s (give the legs the time to reach it! otherwise weird back step!)
        // after that, continue --> compute new towr spline
        if (t > solution.base_linear_->GetTotalTime()){ 

            if (t > (solution.base_linear_->GetTotalTime() + 0))
            {
                flag = false;
                continue; // compute new towr spline
            }

            t = solution.base_linear_->GetTotalTime();

        }
        
        // compute bool array of stance legs. need this to check which legs are in swing phase
        nst = 0;
        for (int i = 0; i < 4; i++){
            contact = solution.phase_durations_.at(array[i])->IsContactPhase(t);
                if(contact == true){
                    nst++;
                    my_check_contact[i] = true;
                } 
                else {
                    my_check_contact[i] = false; 
                }
        }

        // extract right refs from the towr spline
        rc_ref<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
        rc_dot_ref<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
        rc_ddot_ref<<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();

        // some couts. remove when done debugging
        // cout << "actual COM: " << endl << toEigen(CoM) << "\n";
        // cout << "actual vel COM: " << endl << toEigen(CoM_vel) << "\n";
        // cout << "reference CoM" << endl << rc_ref << "\n";
        // cout << "reference vel CoM" << endl << rc_dot_ref << "\n";
        //cout << "reference acc CoM" << endl << rc_ddot_ref << "\n";
        // cout << "robot mass:" << robot_mass <<"\n";

        // set refs for swing feet. the order is BR BL FL FR (same as jacobian)
        // int array[4] = {1,0,2,3};
        temp = 0;
        for (int i = 0; i < 4; i++){ 
            if(my_check_contact[i] == false){
                x_sw_des.block(3*temp,0,3,1) = solution.ee_motion_.at(array[i])->GetPoint(t).p();
                x_sw_dot_des.block(3*temp,0,3,1) = solution.ee_motion_.at(array[i])->GetPoint(t).v();
                x_sw_ddot_des.block(3*temp,0,3,1) = solution.ee_motion_.at(array[i])->GetPoint(t).a();
                temp++;
            }
        }

       // ------------------------------ SOLVE QPPROBLEM -------------------------------
    

       update(_world_H_base, _jnt_pos, _jnt_vel, _base_vel, gravity);
       wcom_des = get_wcom();
       
       // solve problem --> compute motor torques
       if(nst == 4 ) {/*cout << "qpproblem stance \n";*/ tau_star = qpproblem_stance(wcom_des);}
       else if(nst == 2) { /*cout << "qpproblem swing \n";*/ tau_star = qpproblem_swing(wcom_des, my_check_contact);}
       else cout << "Wrong leg number \n";

       // ---------------------------- PLOT STUFF ----------------------------

    //     geometry_msgs::Vector3 _leg_plot;
    //     Eigen::Vector3d eebr = get_eebr_pos();
    //     _leg_plot.x = eebr(0);
    //     _leg_plot.y = eebr(1);
    //     _leg_plot.z = eebr(2);

    //    _leg_plot_pub.publish(_leg_plot); // BR leg pos

    //    // ================

    //     geometry_msgs::Vector3 _leg_ref_plot;
    //     Eigen::Vector3d eebr_ref = solution.ee_motion_.at(1)->GetPoint(t).p();
    //     _leg_ref_plot.x = eebr_ref(0);
    //     _leg_ref_plot.y = eebr_ref(1);
    //     _leg_ref_plot.z = eebr_ref(2);

    //    _leg_ref_plot_pub.publish(_leg_ref_plot); // BR leg ref

    //    // ===============

    //    geometry_msgs::Vector3 _com_plot;
    //     Eigen::Matrix<double,6,1> com__ = toEigen(CoM);
    //     _com_plot.x = com__(0);
    //     _com_plot.y = com__(1);
    //     _com_plot.z = com__(2);

    //    _com_plot_pub.publish(_com_plot); // COM

    //    // ==============

    //    geometry_msgs::Vector3 _com_ref_plot;
    //     Eigen::Matrix<double,3,1> com__ref = solution.base_linear_->GetPoint(t).p();
    //     _com_ref_plot.x = com__ref(0);
    //     _com_ref_plot.y = com__ref(1);
    //     _com_ref_plot.z = com__ref(2);

    //     // std::cout << "\n" <<  _com_ref_plot << "\n";

    //    _com_ref_plot_pub.publish(_com_ref_plot); // COM ref


       // ------------------------------- PUBLISH TORQUES ----------------------

       unpauseGazebo.call(pauseSrv);

       publish_cmd(tau_star);


       // ------------------------------------------------------------------------------------------

        rate.sleep(); 

    } // while(ros::ok())
}

Eigen::Matrix<double,6,1>* Controller::getBasePosPtr() {
    return &_base_pos;
}

void Controller::start() {
    
    ctrl_thread = std::thread(&Controller::ctrl_loop,this);

}