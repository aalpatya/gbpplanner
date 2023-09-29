/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed under MIT license (see LICENSE for details)
/**************************************************************************************/
#pragma once 
#include <memory>
#include <map>
#include <vector>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <Globals.h>
extern Globals globals;
/*************************************************************************************************************/
// This file contains the core algorithm of Gaussian Belief Propagation
/*************************************************************************************************************/
// Message passing can be done within a robot/factorgraph (INTERNAL) or between two different robots/factorgraphs (EXTERNAL)
enum MsgPassingMode {EXTERNAL, INTERNAL};

/***********************************************************************************/
// iterateGBP performs n_iters iterations of message passing of the given message passing mode, on the set of
// factorgraphs provided in factorgraphs. Note, these are in a map, accesed by an int. These could be the map of simulator->robots_.
/***********************************************************************************/
template <typename T>
void iterateGBP(int n_iters, MsgPassingMode msg_passing_mode, std::map<int, std::shared_ptr<T>>& factorgraphs);

template <typename T>
void iterateGBP(int n_iters, MsgPassingMode msg_passing_mode, std::map<int, std::shared_ptr<T>>& factorgraphs){
    for (int i=0; i<n_iters; i++){
#pragma omp parallel for
        // Iterate through robots
        for (int r_idx=0; r_idx<factorgraphs.size(); r_idx++){
            auto it_r = factorgraphs.begin(); std::advance(it_r, r_idx);
            auto& [r_id, factorgraph] = *it_r;
            factorgraph->factorIteration(msg_passing_mode);
        }
    
#pragma omp parallel for
        // Iterate through robots
        for (int r_idx=0; r_idx<factorgraphs.size(); r_idx++){
            auto it_r = factorgraphs.begin(); std::advance(it_r, r_idx);
            auto& [r_id, factorgraph] = *it_r;
            factorgraph->variableIteration(msg_passing_mode);
        }
    }

}    


/******************************************************************/
// This data structure is used to represent both variables and factors
// it includes the id of the robot that the variable/factor belongs to, as well as its own id
// The given operators allow for searching and comparisons using the Key structure
/******************************************************************/
class Key {
    public:
    int robot_id_;
    int node_id_;
    bool valid_;
    
    Key(int graph_id, int node_id, bool valid=true)
        : robot_id_(graph_id), node_id_(node_id), valid_(valid) {}

    friend bool operator== (const Key &key1, const Key &key2) {
        return key1.robot_id_ == key2.robot_id_ && key1.node_id_ == key2.node_id_;
    }
    friend bool operator!= (const Key &key1, const Key &key2) {
        return !(key1.robot_id_ == key2.robot_id_ && key1.node_id_ == key2.node_id_);
    }

    friend bool operator< (const Key &key1, const Key &key2) {
        return (key1.robot_id_ < key2.robot_id_) ||
            (key1.robot_id_ == key2.robot_id_ && key1.node_id_ < key2.node_id_);
    }
};

/******************************************************************/
// Data structure for a message that is passed in GBP
// this consists of an information vector, precision matrix, and a mean vector.
// Traditionally GBP does not require the sending of the last parameter mu (the mean), as it
// can be calculated from the eta and lambda. We include it here for computational efficiency.
/******************************************************************/
class Message {
    public:
    Eigen::VectorXd eta;
    Eigen::MatrixXd lambda;
    Eigen::VectorXd mu;

    // A Message can be initialised with zeros, of the dimension given in the input.
    Message(int n=globals.N_DOFS){
        eta = Eigen::VectorXd::Zero(n);
        lambda = Eigen::MatrixXd::Zero(n, n);
        mu = Eigen::VectorXd::Zero(n);
    }
    // A message can also be initialised explicitly using a given eta, lambda and optionally mu.
    Message(Eigen::VectorXd eta_in, Eigen::MatrixXd lambda_in, Eigen::VectorXd mu_in = Eigen::VectorXd::Zero(globals.N_DOFS)){
        int n = eta_in.rows();
        eta = eta_in;
        lambda = lambda_in;
        mu = (mu_in.rows()==n) ? mu_in : Eigen::VectorXd::Zero(n);
    }
    
    // These operators allow for calculations to be done with Messages, eg. Adding and Subtracting.
    // Note, during addition and subtraction, the mean vector mu is untouched.
    Message& operator+=(const Message& msg_to_add) {eta += msg_to_add.eta; lambda += msg_to_add.lambda; return *this;};
    Message& operator-=(const Message& msg_to_add) {eta -= msg_to_add.eta; lambda -= msg_to_add.lambda; return *this;};
    const Message operator+(const Message& msg_to_add) const {Message ret_msg = *this; ret_msg += msg_to_add; return ret_msg;};
    const Message operator-(const Message& msg_to_sub) const {Message ret_msg = *this; ret_msg -= msg_to_sub; return ret_msg;};
    
    // This function sets the message to zero. mu is untouched.
    void setZero(){
        eta.setZero();
        lambda.setZero();
    }
    // This function sets the mean vector (mu) to a desired value.
    Message& setMu(Eigen::VectorXd mu_in) {this->mu = mu_in; return *this;};
};

// This is the data structure representing a mailbox of Messages, that can be accessed by a Key.
using Mailbox = std::map<Key, Message>;