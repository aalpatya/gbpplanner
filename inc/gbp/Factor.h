/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include "Simulator.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Utils.h>
#include <gbp/GBPCore.h>

#include <raylib.h>

extern Globals globals;

using Eigen::seqN;
using Eigen::seq;
using Eigen::last;

class Variable;     // Forward declaration

// Types of factors defined. Default is DEFAULT_FACTOR
enum FactorType {DEFAULT_FACTOR, DYNAMICS_FACTOR, INTERROBOT_FACTOR, OBSTACLE_FACTOR};
/*****************************************************************************************/
// Factor used in GBP
/*****************************************************************************************/
class Factor {
    public:
    Simulator* sim_;                            // Pointer to simulator
    int f_id_;                                  // Factor id
    int r_id_;                                  // Robot id this factor belongs to
    Key key_;                                   // Factor key = {r_id_, f_id_}
    int other_rid_;                             // id of other connected robot (if this is an inter-robot factor)
    int n_dofs_;                                // n_dofs of the variables connected to this factor
    Eigen::VectorXd z_;                         // Measurement
    Eigen::MatrixXd h_, J_;                     // Stored values of measurement function h_func_() and Jacobian J_func_()
    Eigen::VectorXd X_;                         // Stored linearisation point
    Eigen::MatrixXd meas_model_lambda_;         // Precision of measurement model
    Mailbox inbox_, outbox_, last_outbox_;      
    FactorType factor_type_ = DEFAULT_FACTOR; 
    float delta_jac=1e-8;                       // Delta used for first order jacobian calculation
    bool initialised_ = false;                  // Becomes true when Jacobian calculated for the first time
    bool linear_ = false;                       // True is factor is linear (avoids recomputation of Jacobian)
    bool skip_flag = false;                          // Flag to skip factor update if required
    virtual bool skip_factor(){                 // Default function to set skip flag
        skip_flag = false;
        return skip_flag;
    };
    std::vector<std::shared_ptr<Variable>> variables_{};    // Vector of pointers to the connected variables. Order of variables matters

    

    // Function declarations
    Factor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
            float sigma, const Eigen::VectorXd& measurement, 
            int n_dofs=4);

    ~Factor();

    void draw();

    virtual Eigen::MatrixXd h_func_(const Eigen::VectorXd& X) = 0;

    virtual Eigen::MatrixXd J_func_(const Eigen::VectorXd& X);

    Eigen::MatrixXd jacobianFirstOrder(const Eigen::VectorXd& X0);

    virtual Eigen::VectorXd residual(){return z_ - h_;};

    bool update_factor();

    Message marginalise_factor_dist(const Eigen::VectorXd &eta, const Eigen::MatrixXd &Lam, int var_idx, int marg_idx);
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
class DynamicsFactor: public Factor {
    public:

    DynamicsFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
        float sigma, const Eigen::VectorXd& measurement, float dt);

    // Constant velocity model
    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X);
    Eigen::MatrixXd J_func_(const Eigen::VectorXd& X);

};

/********************************************************************************************/
/* Interrobot factor: for avoidance of other robots */
// This factor results in a high energy or cost if two robots are planning to be in the same 
// position at the same timestep (collision). This factor is created between variables of two robots.
// The factor has 0 energy if the variables are further away than the safety distance. skip_ = true in this case.
/********************************************************************************************/
class InterrobotFactor: public Factor {
    public:
    double safety_distance_;

    InterrobotFactor(int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
        float sigma, const Eigen::VectorXd& measurement, 
        float robot_radius);

    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X);
    Eigen::MatrixXd J_func_(const Eigen::VectorXd& X);
    bool skip_factor();

};

/********************************************************************************************/
// Obstacle factor for static obstacles in the scene. This factor takes a pointer to the obstacle image from the Simulator.
// Note. in the obstacle image, white areas represent obstacles (as they have a value of 1).
// The input image to the simulator is opposite, which is why it needs to be inverted.
// The delta used in the first order jacobian calculation is chosen such that it represents one pixel in the image.
/********************************************************************************************/
class ObstacleFactor: public Factor {
    public:
    Image* p_obstacleImage_;

    ObstacleFactor(Simulator* sim, int f_id, int r_id, std::vector<std::shared_ptr<Variable>> variables,
        float sigma, const Eigen::VectorXd& measurement, Image* p_obstacleImage);

    Eigen::MatrixXd h_func_(const Eigen::VectorXd& X);

};
