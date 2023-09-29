/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed under MIT license (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <memory>
#include <map>

#include <Utils.h>
#include <gbp/GBPCore.h>
#include <gbp/Factor.h>
#include <gbp/Variable.h>

#include <Eigen/Dense>
#include <Eigen/Core>

/******************************************************************************************************/
// The robot class derives from this FactorGraph class.
// It contains variables and factors, stored in maps, indexed by their Keys (robot_id, node_id)
/******************************************************************************************************/
class FactorGraph {
    public:
    int robot_id_;                                          // id of the robot this belongs to
    std::map<Key, std::shared_ptr<Variable>> variables_{};
    std::map<Key, std::shared_ptr<Factor>> factors_{};
    bool interrobot_comms_active_ = true;                   // Flag for whether this factorgraph/robot communicates with other robots

    // GBP functions
    void factorIteration(MsgPassingMode msg_passing_mode);  
    void variableIteration(MsgPassingMode msg_passing_mode);

    // Constructor
    FactorGraph(int robot_id);

    // Access the i'th variable within this factorgraph as :
    // auto variable = factorgraph->getVar(i); Returns smart pointer ref to the variable
    std::shared_ptr<Variable>& getVar(const int& i){
        int n = variables_.size();
        int search_vid = ((n + i) % n + n) % n;

        auto it = variables_.begin();
        std::advance(it, search_vid);
        return it->second;
    }
    
    // Access the variable by a specific key
    std::shared_ptr<Variable>& getVar(const Key& v_key){
        return variables_[v_key];
    }

};
