#include "topt.h"

void get_trajectory(Eigen::VectorXd com_p, Eigen::VectorXd com_dp, Eigen::VectorXd dcom_p, Eigen::Vector3d eeBLpos, Eigen::Vector3d eeBRpos, 
    Eigen::Vector3d eeFLpos, Eigen::Vector3d eeFRpos, int gait_flag, float duration, towr::SplineHolder & solution, towr::NlpFormulation & formulation_ ) {

    std::printf("\n \nGoing from [%f %f %f | %f] to [%f %f %f | %f] \n \n ", com_p(0), com_p(1), com_p(2), com_p(5),dcom_p(0),dcom_p(1),dcom_p(2),dcom_p(5));
    
    Eigen::Vector2d delta;
    delta = dcom_p.head(2) - com_p.head(2);
    std::printf(" Step length: %f \n \n", sqrt(delta.squaredNorm()));

    formulation_.terrain_ = std::make_shared<towr::FlatGround>(0.0);
    formulation_.model_ = towr::RobotModel(towr::RobotModel::Dogbot);


    formulation_.initial_base_.lin.at(towr::kPos) << com_p(0), com_p(1), com_p(2); 
    // formulation_.initial_base_.ang.at(towr::kPos) << com_p(3), com_p(4), com_p(5);
    formulation_.initial_base_.lin.at(towr::kVel) << com_dp(0), com_dp(1), com_dp(2); 
    formulation_.initial_base_.ang.at(towr::kVel) << com_dp(3), com_dp(4), com_dp(5); 

    // special handling for yaw:
    // maybe it works even without the sign check (only difference)
    if(abs(dcom_p(5)-com_p(5)) > M_PI && (dcom_p(5)*com_p(5)) < 0){ // if different sign and difference > 180
        formulation_.initial_base_.ang.at(towr::kPos) << com_p(3), com_p(4), com_p(5) + 2*M_PI; 
    }
    else { // regular case
        formulation_.initial_base_.ang.at(towr::kPos) << com_p(3), com_p(4), com_p(5); 

    }

    formulation_.final_base_.lin.at(towr::kPos) << dcom_p(0), dcom_p(1), dcom_p(2); 
    formulation_.final_base_.ang.at(towr::kPos) << dcom_p(3), dcom_p(4), dcom_p(5);
    // formulation_.final_base_.ang.at(towr::kPos) << dcom_p(3), dcom_p(4), com_p(5) + 0.1; // to make the robot spin on z



  
//Trajectory
//    if((ros::Time::now()).toSec()>7 && (ros::Time::now()).toSec()<17)
//     {formulation_.final_base_.lin.at(towr::kPos) << formulation_.initial_base_.lin.at(towr::kPos)[0]+0.04*std::sin(formulation_.initial_base_.ang.at(towr::kPos)[2]), formulation_.initial_base_.lin.at(towr::kPos)[1]-0.04*std::cos(formulation_.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
//      formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, formulation_.initial_base_.ang.at(towr::kPos)[2]-0.025;}
//    else if((ros::Time::now()).toSec()<7){
//      formulation_.final_base_.lin.at(towr::kPos) << 0.0, formulation_.initial_base_.lin.at(towr::kPos)[1]-0.05, 0.40229;
//      formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, 0.0;
//    }


    auto nominal_stance_B = formulation_.model_.kinematic_model_->GetNominalStanceInBase();
    formulation_.initial_ee_W_ = nominal_stance_B;  

    formulation_.initial_ee_W_.at(0)[0]=eeBLpos[0];
    formulation_.initial_ee_W_.at(0)[1]=eeBLpos[1];
    formulation_.initial_ee_W_.at(1)[0]=eeBRpos[0];
    formulation_.initial_ee_W_.at(1)[1]=eeBRpos[1];
    formulation_.initial_ee_W_.at(2)[0]=eeFLpos[0];
    formulation_.initial_ee_W_.at(2)[1]=eeFLpos[1];
    formulation_.initial_ee_W_.at(3)[0]=eeFRpos[0];
    formulation_.initial_ee_W_.at(3)[1]=eeFRpos[1];

   std::for_each(formulation_.initial_ee_W_.begin(), formulation_.initial_ee_W_.end(), [&](Eigen::Vector3d& p) { 
        p[2]= 0.0; 
        } 
    );
  
  
    // Choose gait (se scegli un flag che non c'è dovrebbe assegnare di default solo stand)
    auto gait_gen_ = towr::GaitGenerator::MakeGaitGenerator(4);
    if (gait_flag == 1) {
        auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C1);
        gait_gen_->SetCombo(id_gait);
    }
    else if (gait_flag == 2) {
        auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C5);
        gait_gen_->SetCombo(id_gait);
    }
    else if (gait_flag == 3) {
        auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C6);
        gait_gen_->SetCombo(id_gait);
    }


   formulation_.params_.ee_phase_durations_.clear();
   for (int ee=0; ee<4; ++ee) {
      formulation_.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(duration, ee));
      formulation_.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));      
    }


    ifopt::Problem nlp;

    // Compute solution
    for (auto c : formulation_.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for (auto c : formulation_.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for (auto c : formulation_.GetCosts())
        nlp.AddCostSet(c);

    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
    solver->SetOption("max_cpu_time", 20.0);
    solver->Solve(nlp);

}




