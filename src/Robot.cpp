/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed under MIT license (see LICENSE for details)
/**************************************************************************************/
#include <Robot.h>

/***************************************************************************/
// Creates a robot. Inputs required are :
//      - Pointer to the simulator
//      - A robot id rid (should be taken from simulator->next_rid_++),
//      - A dequeue of waypoints (which are 4 dimensional [x,y,xdot,ydot])
//      - Robot radius
//      - Colour
/***************************************************************************/
Robot::Robot(Simulator* sim,
             int rid,
             std::deque<Eigen::VectorXd> waypoints,
             float size,
             Color color) : FactorGraph{sim->next_rid_},
             sim_(sim), rid_(rid),
             waypoints_(waypoints),
             robot_radius_(size), color_(color) {

    height_3D_ = robot_radius_;     // Height out of plane for 3d visualisation only

    // Robot will always set its horizon state to move towards the next waypoint.
    // Once this waypoint has been reached, it pops it from the waypoints
    Eigen::VectorXd start = position_ = waypoints_[0];
    waypoints_.pop_front();                             
    auto goal = (waypoints_.size()>0) ? waypoints_[0] : start;

    // Initialise the horzion in the direction of the goal, at a distance T_HORIZON * MAX_SPEED from the start.
    Eigen::VectorXd start2goal = goal - start;
    Eigen::VectorXd horizon = start + std::min(start2goal.norm(), 1.*globals.T_HORIZON*globals.MAX_SPEED)*start2goal.normalized();

    // Variables representing the planned path are at timesteps which increase in spacing.
    // eg. (so that a span of 10 timesteps as a planning horizon can be represented by much fewer variables)
    std::vector<int> variable_timesteps = getVariableTimesteps(globals.T_HORIZON / globals.T0, globals.LOOKAHEAD_MULTIPLE);
    num_variables_ = variable_timesteps.size();

    /***************************************************************************/
    /* Create Variables with fixed pose priors on start and horizon variables. */
    /***************************************************************************/
    Color var_color = color_; double sigma; int n = globals.N_DOFS;
    Eigen::VectorXd mu(n); Eigen::VectorXd sigma_list(n); 
    for (int i = 0; i < num_variables_; i++){
        // Set initial mu and covariance of variable interpolated between start and horizon
        mu = start + (horizon - start) * (float)(variable_timesteps[i]/(float)variable_timesteps.back());
        // Start and Horizon state variables should be 'fixed' during optimisation at a timestep
        sigma = (i==0 || i==num_variables_-1) ? globals.SIGMA_POSE_FIXED : 0.;
        sigma_list.setConstant(sigma);
        
        // Create variable and add to robot's factor graph 
        auto variable = std::make_shared<Variable>(sim->next_vid_++, rid_, mu, sigma_list, robot_radius_, n);
        variables_[variable->key_] = variable;
    }

    /***************************************************************************/
    /* Create Dynamics factors between variables */
    /***************************************************************************/
    for (int i = 0; i < num_variables_-1; i++)
    {
        // T0 is the timestep between the current state and the first planned state.
        float delta_t = globals.T0 * (variable_timesteps[i + 1] - variable_timesteps[i]);
        std::vector<std::shared_ptr<Variable>> variables {getVar(i), getVar(i+1)};
        auto factor = std::make_shared<DynamicsFactor>(sim->next_fid_++, rid_, variables, globals.SIGMA_FACTOR_DYNAMICS, Eigen::VectorXd::Zero(globals.N_DOFS), delta_t);
        
        // Add this factor to the variable's list of adjacent factors, as well as to the robot's list of factors
        for (auto var : factor->variables_) var->add_factor(factor);
        factors_[factor->key_] = factor;
    }

    /***************************************************************************/
    // Create Obstacle factors for all variables excluding start, excluding horizon
    /***************************************************************************/
    for (int i = 1; i < num_variables_-1; i++)
    {
        std::vector<std::shared_ptr<Variable>> variables{getVar(i)};
        auto fac_obs = std::make_shared<ObstacleFactor>(sim, sim->next_fid_++, rid_, variables, globals.SIGMA_FACTOR_OBSTACLE, Eigen::VectorXd::Zero(1), &(sim_->obstacleImg));

        // Add this factor to the variable's list of adjacent factors, as well as to the robot's list of factors
        for (auto var : fac_obs->variables_) var->add_factor(fac_obs);
        this->factors_[fac_obs->key_] = fac_obs;
    }

};

/***************************************************************************************************/
/* Destructor */
/***************************************************************************************************/
Robot::~Robot(){
}

/***************************************************************************************************/
/* Change the prior of the Current state */
/***************************************************************************************************/
void Robot::updateCurrent(){
    // Move plan: move plan current state by plan increment
    Eigen::VectorXd increment = ((*this)[1]->mu_ - (*this)[0]->mu_) * globals.TIMESTEP / globals.T0;
    // In GBP we do this by modifying the prior on the variable
    getVar(0)->change_variable_prior(getVar(0)->mu_ + increment);
    // Real pose update
    position_ = position_ + increment;

};

/***************************************************************************************************/
/* Change the prior of the Horizon state */
/***************************************************************************************************/
void Robot::updateHorizon(){
    // Horizon state moves towards the next waypoint.
    // The Horizon state's velocity is capped at MAX_SPEED
    auto horizon = getVar(-1);      // get horizon state variable
    Eigen::VectorXd dist_horz_to_goal = waypoints_.front()({0,1}) - horizon->mu_({0,1});                        
    Eigen::VectorXd new_vel = dist_horz_to_goal.normalized() * std::min((double)globals.MAX_SPEED, dist_horz_to_goal.norm());
    Eigen::VectorXd new_pos = horizon->mu_({0,1}) + new_vel*globals.TIMESTEP;
    
    // Update horizon state with new pos and vel
    horizon->mu_ << new_pos, new_vel;
    horizon->change_variable_prior(horizon->mu_);

    // If the horizon has reached the waypoint, pop that waypoint from the waypoints.
    // Could add other waypoint behaviours here (maybe they might move, or change randomly).
    if (dist_horz_to_goal.norm() < robot_radius_){
        if (waypoints_.size()>1) waypoints_.pop_front();
    }
}


/***************************************************************************************************/
// For new neighbours of a robot, create inter-robot factors if they don't exist. 
// Delete existing inter-robot factors for faraway robots
/***************************************************************************************************/
void Robot::updateInterrobotFactors(){
    
    // Search through currently connected rids. If any are not in neighbours, delete interrobot factors.
    for (auto rid : connected_r_ids_){
        if (std::find(neighbours_.begin(), neighbours_.end(), rid)==neighbours_.end()){
            deleteInterrobotFactors(sim_->robots_.at(rid));
        };
    }
    // Search through neighbours. If any are not in currently connected rids, create interrobot factors.
    for (auto rid : neighbours_){
        if (std::find(connected_r_ids_.begin(), connected_r_ids_.end(), rid)==connected_r_ids_.end()){
            createInterrobotFactors(sim_->robots_.at(rid));
            if (!sim_->symmetric_factors) sim_->robots_.at(rid)->connected_r_ids_.push_back(rid_);
        };
    }
}

/***************************************************************************************************/
// Create inter-robot factors between this robot and another robot
/***************************************************************************************************/
void Robot::createInterrobotFactors(std::shared_ptr<Robot> other_robot)
{
    // Create Interrobot factors for all timesteps excluding current state
    for (int i = 1; i < num_variables_; i++){
        // Get variables
        std::vector<std::shared_ptr<Variable>> variables{getVar(i), other_robot->getVar(i)};

        // Create the inter-robot factor
        Eigen::VectorXd z = Eigen::VectorXd::Zero(variables.front()->n_dofs_);
        auto factor = std::make_shared<InterrobotFactor>(sim_->next_fid_++, this->rid_, variables, globals.SIGMA_FACTOR_INTERROBOT, z, 0.5*(this->robot_radius_ + other_robot->robot_radius_));
        factor->other_rid_ = other_robot->rid_;
        // Add factor the the variable's list of factors, as well as to the robot's list of factors
        for (auto var : factor->variables_) var->add_factor(factor);
        this->factors_[factor->key_] = factor;
    }

    // Add the other robot to this robot's list of connected robots.
    this->connected_r_ids_.push_back(other_robot->rid_);
};

/***************************************************************************************************/
/* Delete interrobot factors between the two robots */
/***************************************************************************************************/
void Robot::deleteInterrobotFactors(std::shared_ptr<Robot> other_robot)
{
    std::vector<Key> facs_to_delete{};
    for (auto& [f_key, fac] : this->factors_){
        if (fac->other_rid_ != other_robot->rid_) continue;

        // Only get here if factor is connected to a variable in the other_robot
        for (auto& var : fac->variables_){ 
            var->delete_factor(f_key);
            facs_to_delete.push_back(f_key);
        }
    }
    for (auto f_key : facs_to_delete) this->factors_.erase(f_key);

    // Remove other robot from current robot's connected rids
    auto it = std::find(connected_r_ids_.begin(), connected_r_ids_.end(), other_robot->rid_);
    if (it != connected_r_ids_.end()){
        connected_r_ids_.erase(it);
    }

};

/***************************************************************************************************/
// Drawing functions for the robot.
// We deal with a 2d problem, so the out-of-plane height is set to height_3D_.
/***************************************************************************************************/
void Robot::draw(){
    Color col = (interrobot_comms_active_) ? color_ : GRAY;
    // Draw planned path
    if (globals.DRAW_PATH){
        static int debug = 0;
        for (auto [vid, variable] : variables_){
            if (!variable->valid_) continue;
            DrawSphere(Vector3{(float)variable->mu_(0), height_3D_, (float)variable->mu_(1)}, 0.5*robot_radius_, ColorAlpha(col, 0.5));
        }
        for (auto [fid, factor] : factors_) factor->draw();
    }     
    // Draw connected robots
    if (globals.DRAW_INTERROBOT){
        for (auto rid : connected_r_ids_){
            if (!interrobot_comms_active_ || !sim_->robots_.at(rid)->interrobot_comms_active_) continue;
            DrawCylinderEx(Vector3{(float)position_(0), height_3D_, (float)position_(1)},
                            Vector3{(float)(*sim_->robots_.at(rid))[0]->mu_(0), sim_->robots_.at(rid)->height_3D_, (float)(*sim_->robots_.at(rid))[0]->mu_(1)}, 
                            0.1, 0.1, 4, BLACK);
        }
    }

    // Draw the waypoints of the robot
    if (globals.DRAW_WAYPOINTS){
        for (int wp_idx=0; wp_idx<waypoints_.size(); wp_idx++){
            DrawCubeV(Vector3{(float)waypoints_[wp_idx](0), height_3D_, (float)waypoints_[wp_idx](1)}, Vector3{1.f*robot_radius_,1.f*robot_radius_,1.f*robot_radius_}, col);
        }
    }
    // Draw the actual position of the robot. This uses the robotModel defined in Graphics.cpp, others can be used.
    DrawModel(sim_->graphics->robotModel_, Vector3{(float)position_(0), height_3D_, (float)position_(1)}, robot_radius_, col);
};

/*******************************************************************************************/
// Function for determining the timesteps at which variables in the planned path are placed.
/*******************************************************************************************/
std::vector<int> Robot::getVariableTimesteps(int lookahead_horizon, int lookahead_multiple){
    // For a lookahead_multiple of 3, variables are spaced at timesteps:
    // Timesteps
    // 0,    1, 2, 3,    5, 7, 9,    12, 15, 18, ...
    // 
    // eg. variables are in groups of size lookahead_multiple.
    // the spacing within a group increases by one each time (1 for the first group, 2 for the second ...)
    // Seems convoluted, but the reasoning was:
    //      the first variable should always be at 1 timestep from the current state (0).
    //      the first few variables should be close together in time
    //      the variables should all be at integer timesteps, but the spacing should sort of increase exponentially.
    std::vector<int> var_list{};
    int N = 1 + int(0.5*(-1 + sqrt(1 + 8*(float)lookahead_horizon/(float)lookahead_multiple)));

    for (int i=0; i<lookahead_multiple*(N+1); i++){
        int section = int(i/lookahead_multiple);
        int f = (i - section*lookahead_multiple + lookahead_multiple/2.*section)*(section+1);
        if (f>=lookahead_horizon){
            var_list.push_back(lookahead_horizon);
            break;
        }
        var_list.push_back(f);
    }

    return var_list;
};