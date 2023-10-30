/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#include <Utils.h>
#include <gbp/GBPCore.h>
#include <gbp/Factor.h>
#include <gbp/Variable.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <raylib.h>

/*****************************************************************************************************/
// Factor constructor
// Inputs:
//  - factor id (taken from simulator->next_f_id_++)
//  - robot id that this factor belongs to.
//  - A vector of pointers to Variables that the factor is to be connected to. Note, the order of the variables matters.
//  - sigma: factor strength. The factor precision Lambda = sigma^-2 * Identity
//  - measurement z: Eigen::VectorXd, must be same size as the output of the measurement function h().
//  - n_dofs is the number of degrees of freedom of the variables this factor is connected to. (eg. 4 for [x,y,xdot,ydot])
/*****************************************************************************************************/
Factor::Factor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
        float sigma, const Eigen::VectorXd& measurement, 
        int n_dofs) 
        : f_id_(f_id), r_id_(r_id), key_(r_id, f_id), variables_(variables), z_(measurement), n_dofs_(n_dofs) {

        // Initialise precision of the measurement function
        this->meas_model_lambda_ = Eigen::MatrixXd::Identity(z_.rows(), z_.rows()) / pow(sigma,2.);
        
        // Initialise empty inbox and outbox
        int n_dofs_total = 0; int n_dofs_var;
        for (auto var : variables_) {
            n_dofs_var = var->n_dofs_;
            Message zero_msg(n_dofs_var);
            inbox_[var->key_] = zero_msg;
            outbox_[var->key_] = zero_msg;
            n_dofs_total += n_dofs_var;
        }

        // This parameter useful if the factor is connected to another robot
        other_rid_=r_id_;                           

        // Initialise empty linearisation point
        X_ = Eigen::VectorXd::Zero(n_dofs_total);
    };

/*****************************************************************************************************/
// Destructor
/*****************************************************************************************************/
Factor::~Factor(){
}

/*****************************************************************************************************/
// Drawing function for the factor. Draws a 3d Cylinder (line-ish) between its connected variables
/*****************************************************************************************************/
void Factor::draw(){
    if ((factor_type_==DYNAMICS_FACTOR && globals.DRAW_PATH)){
        auto v_0 = variables_[0];
        auto v_1 = variables_[1];
        if (!v_0->valid_ || !v_1->valid_) {return;};
        DrawCylinderEx(Vector3{(float)v_0->mu_(0), globals.ROBOT_RADIUS, (float)v_0->mu_(1)},
                        Vector3{(float)v_1->mu_(0), globals.ROBOT_RADIUS, (float)v_1->mu_(1)}, 
                        0.1, 0.1, 4, BLACK);        
    }    
}

/*****************************************************************************************************/
// Default measurement function h_func_() is the identity function: it returns the variable.
/*****************************************************************************************************/
Eigen::MatrixXd h_func_(const Eigen::VectorXd& X){return X;};
/*****************************************************************************************************/
// Default measurement function Jacobian J_func_() is the first order taylor series jacobian by default.
// When defining new factors, custom h_func_() and J_func_() must be defined, otherwise defaults are used.
/*****************************************************************************************************/
Eigen::MatrixXd Factor::J_func_(const Eigen::VectorXd& X){return this->jacobianFirstOrder(X);};

Eigen::MatrixXd Factor::jacobianFirstOrder(const Eigen::VectorXd& X0){
    Eigen::MatrixXd h0 = h_func_(X0);    // Value at lin point
    Eigen::MatrixXd jac_out = Eigen::MatrixXd::Zero(h0.size(),X0.size());
    for (int i=0; i<X0.size(); i++){
        Eigen::VectorXd X_copy = X0;                                    // Copy of lin point
        X_copy(i) += delta_jac;                                         // Perturb by delta
        jac_out(Eigen::all, i) = (h_func_(X_copy) - h0) / delta_jac;    // Derivative (first order)
    }
    return jac_out;
};

/*****************************************************************************************************/
// Main section: Factor update:
// Messages from connected variables are aggregated. The beliefs are used to create the linearisation point X_.
// The Factor potential is calculated using h_func_ and J_func_
// The factor precision and information is created, and then marginalised to create outgoing messages to its connected variables.
/*****************************************************************************************************/
bool Factor::update_factor(){

    // Messages from connected variables are aggregated.
    // The beliefs are used to create the linearisation point X_.
    int idx = 0; int n_dofs;
    for (int v=0; v<variables_.size(); v++){
        n_dofs = variables_[v]->n_dofs_;
        auto& [_, __, mu_belief] = this->inbox_[variables_[v]->key_];
        X_(seqN(idx, n_dofs)) = mu_belief;
        idx += n_dofs;
    }

    // *Depending on the problem*, we may need to skip computation of this factor.
    // eg. to avoid extra computation, factor may not be required if two connected variables are too far apart.
    // in which case send out a Zero Message.
    if (this->skip_factor()){
        for (auto var : variables_){
            this->outbox_[var->key_] = Message(var->n_dofs_);
        }           
        return false;
    }
    
    // The Factor potential and linearised Factor Precision and Information is calculated using h_func_ and J_func_
    // residual() is by default (z - h_func_(X))
    // Skip calculation of Jacobian if the factor is linear and Jacobian has already been computed once
    h_ = h_func_(X_);
    J_ = (this->linear_ && this->initialised_)? J_ : this->J_func_(X_);
    Eigen::MatrixXd factor_lam_potential = J_.transpose() * meas_model_lambda_ * J_;
    Eigen::VectorXd factor_eta_potential = (J_.transpose() * meas_model_lambda_) * (J_ * X_ + residual());
    this->initialised_ = true;

    //  Update factor precision and information with incoming messages from connected variables.
    int marginalisation_idx = 0;
    for (int v_out_idx=0; v_out_idx<variables_.size(); v_out_idx++){
        auto var_out = variables_[v_out_idx];
        // Initialise with factor values
        Eigen::VectorXd factor_eta = factor_eta_potential;     
        Eigen::MatrixXd factor_lam = factor_lam_potential;
        
        // Combine the factor with the belief from other variables apart from the receiving variable
        int idx_v = 0;
        for (int v_idx=0; v_idx<variables_.size(); v_idx++){
            int n_dofs = variables_[v_idx]->n_dofs_;
            if (variables_[v_idx]->key_ != var_out->key_) {
                auto [eta_belief, lam_belief, _] = inbox_[variables_[v_idx]->key_];
                factor_eta(seqN(idx_v, n_dofs)) += eta_belief;
                factor_lam(seqN(idx_v, n_dofs), seqN(idx_v, n_dofs)) += lam_belief;
            }
            idx_v += n_dofs;
        }
        
        // Marginalise the Factor Precision and Information to send to the relevant variable
        outbox_[var_out->key_] = marginalise_factor_dist(factor_eta, factor_lam, v_out_idx, marginalisation_idx);
        marginalisation_idx += var_out->n_dofs_;
    }

    return true;
};

/*****************************************************************************************************/
// Marginalise the factor Precision and Information and create the outgoing message to the variable
/*****************************************************************************************************/
Message Factor::marginalise_factor_dist(const Eigen::VectorXd &eta, const Eigen::MatrixXd &Lam, int var_idx, int marg_idx){
    // Marginalisation only needed if factor is connected to >1 variables
    int n_dofs = variables_[var_idx]->n_dofs_;
    if (eta.size() == n_dofs) return Message {eta, Lam};

    Eigen::VectorXd eta_a(n_dofs), eta_b(eta.size()-n_dofs);
    eta_a = eta(seqN(marg_idx, n_dofs));
    eta_b << eta(seq(0, marg_idx - 1)), eta(seq(marg_idx + n_dofs, last));

    Eigen::MatrixXd lam_aa(n_dofs, n_dofs), lam_ab(n_dofs, Lam.cols()-n_dofs);
    Eigen::MatrixXd lam_ba(Lam.rows()-n_dofs, n_dofs), lam_bb(Lam.rows()-n_dofs, Lam.cols()-n_dofs);
    lam_aa << Lam(seqN(marg_idx, n_dofs), seqN(marg_idx, n_dofs));
    lam_ab << Lam(seqN(marg_idx, n_dofs), seq(0, marg_idx - 1)), Lam(seqN(marg_idx, n_dofs), seq(marg_idx + n_dofs, last));
    lam_ba << Lam(seq(0, marg_idx - 1), seq(marg_idx, marg_idx + n_dofs - 1)), Lam(seq(marg_idx + n_dofs, last), seqN(marg_idx, n_dofs));
    lam_bb << Lam(seq(0, marg_idx - 1), seq(0, marg_idx - 1)), Lam(seq(0, marg_idx - 1), seq(marg_idx + n_dofs, last)),
            Lam(seq(marg_idx + n_dofs, last), seq(0, marg_idx - 1)), Lam(seq(marg_idx + n_dofs, last), seq(marg_idx + n_dofs, last));

    Eigen::MatrixXd lam_bb_inv = lam_bb.inverse();
    Message marginalised_msg(n_dofs);
    marginalised_msg.eta = eta_a - lam_ab * lam_bb_inv * eta_b;
    marginalised_msg.lambda = lam_aa - lam_ab * lam_bb_inv * lam_ba;
    if (!marginalised_msg.lambda.allFinite()) marginalised_msg.setZero();

    return marginalised_msg;
};    

/********************************************************************************************/
/********************************************************************************************/
//                      CUSTOM FACTORS SPECIFIC TO THE PROBLEM
// Create a new factor definition as shown with these examples.
// You may create a new factor_type_, in the enum in Factor.h (optional, default type is DEFAULT_FACTOR)
// Create a measurement function h_func_() and optionally Jacobian J_func_().

/********************************************************************************************/
/* Dynamics factor: constant-velocity model */
/*****************************************************************************************************/
DynamicsFactor::DynamicsFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
    float sigma, const Eigen::VectorXd& measurement, 
    float dt)
    : Factor{f_id, r_id, variables, sigma, measurement}{ 
        factor_type_ = DYNAMICS_FACTOR;
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_dofs_/2,n_dofs_/2);
        Eigen::MatrixXd O = Eigen::MatrixXd::Zero(n_dofs_/2,n_dofs_/2);
        Eigen::MatrixXd Qc_inv = pow(sigma, -2.) * I;

        Eigen::MatrixXd Qi_inv(n_dofs_, n_dofs_);
        Qi_inv << 12.*pow(dt, -3.) * Qc_inv,   -6.*pow(dt, -2.) * Qc_inv,
                  -6.*pow(dt, -2.) * Qc_inv,   4./dt * Qc_inv;   

        this->meas_model_lambda_ = Qi_inv;        

        // Store Jacobian as it is linear
        this->linear_ = true;
        J_ = Eigen::MatrixXd::Zero(n_dofs_, n_dofs_*2);
        J_ << I, dt*I, -1*I,    O,
             O,    I,    O, -1*I; 

    };

Eigen::MatrixXd DynamicsFactor::h_func_(const Eigen::VectorXd& X){
    return J_ * X;
}    
Eigen::MatrixXd DynamicsFactor::J_func_(const Eigen::VectorXd& X){
    return J_;
}

/********************************************************************************************/
/* Interrobot factor: for avoidance of other robots */
// This factor results in a high energy or cost if two robots are planning to be in the same 
// position at the same timestep (collision). This factor is created between variables of two robots.
// The factor has 0 energy if the variables are further away than the safety distance. skip_ = true in this case.
/********************************************************************************************/

InterrobotFactor::InterrobotFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
    float sigma, const Eigen::VectorXd& measurement, 
    float robot_radius)
    : Factor{f_id, r_id, variables, sigma, measurement} {  
        factor_type_ = INTERROBOT_FACTOR;
        float eps = 0.2 * robot_radius;
        this->safety_distance_ = 2*robot_radius + eps;
        this->delta_jac = 1e-2;
};

Eigen::MatrixXd InterrobotFactor::h_func_(const Eigen::VectorXd& X){
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(z_.rows(),z_.cols());
    Eigen::VectorXd X_diff = X(seqN(0,n_dofs_/2)) - X(seqN(n_dofs_, n_dofs_/2));
    X_diff += 1e-6*r_id_*Eigen::VectorXd::Ones(n_dofs_/2);

    double r = X_diff.norm();
    if (r <= safety_distance_){
        this->skip_flag = false;
        h(0) = 1.f*(1 - r/safety_distance_);
    }
    else {
        this->skip_flag = true;
    }

    return h;
};

Eigen::MatrixXd InterrobotFactor::J_func_(const Eigen::VectorXd& X){
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(z_.rows(), n_dofs_*2);
    Eigen::VectorXd X_diff = X(seqN(0,n_dofs_/2)) - X(seqN(n_dofs_, n_dofs_/2));
    X_diff += 1e-6*r_id_*Eigen::VectorXd::Ones(n_dofs_/2);// Add a tiny random offset to avoid div/0 errors
    double r = X_diff.norm();
    if (r <= safety_distance_){
        J(0,seqN(0, n_dofs_/2)) = -1.f/safety_distance_/r * X_diff;
        J(0,seqN(n_dofs_, n_dofs_/2)) = 1.f/safety_distance_/r * X_diff;
    }
    return J;
};

bool InterrobotFactor::skip_factor(){
    this->skip_flag = ( (X_(seqN(0,n_dofs_/2)) - X_(seqN(n_dofs_, n_dofs_/2))).squaredNorm() >= safety_distance_*safety_distance_ );
    return this->skip_flag;
}


/********************************************************************************************/
// Obstacle factor for static obstacles in the scene. This factor takes a pointer to the obstacle image from the Simulator.
// Note. in the obstacle image, white areas represent obstacles (as they have a value of 1).
// The input image to the simulator is opposite, which is why it needs to be inverted.
// The delta used in the first order jacobian calculation is chosen such that it represents one pixel in the image.
/********************************************************************************************/
ObstacleFactor::ObstacleFactor(Simulator* sim, int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
    float sigma, const Eigen::VectorXd& measurement, Image* p_obstacleImage)
    : Factor{f_id, r_id, variables, sigma, measurement}, p_obstacleImage_(p_obstacleImage){
        factor_type_ = OBSTACLE_FACTOR;
        this->delta_jac = 1.*(float)globals.WORLD_SZ / (float)p_obstacleImage->width;
};
Eigen::MatrixXd ObstacleFactor::h_func_(const Eigen::VectorXd& X){
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(1,1);
    // White areas are obstacles, so h(0) should return a 1 for these regions.
    float scale = p_obstacleImage_->width / (float)globals.WORLD_SZ;
    Vector3 c_hsv = ColorToHSV(GetImageColor(*p_obstacleImage_, (int)((X(0) + globals.WORLD_SZ/2) * scale), (int)((X(1) + globals.WORLD_SZ/2) * scale)));
    h(0) = c_hsv.z;
    return h;
};