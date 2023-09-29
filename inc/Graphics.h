/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed under MIT license (see LICENSE for details)
/**************************************************************************************/
#pragma once
#include <Simulator.h>
#include <raymath.h>
#include <rcamera.h>

/**************************************************************************/
// Graphics class that deals with the nitty-gritty of display.
// Camera is also included here. You can set different camera positions/trajectories
// and then during simulation cycle through them using the SPACEBAR

// Please note: Raylib camera defines the world with positive X = right, positive Z = down, and positive Y = out-of-plane
// But in our work we use the standard convention of positive X = right, positive Y = down, and positive Z = into-plane
/**************************************************************************/
class Graphics {
public:
    // Constructor
    Graphics(Image obstacleImg);
    ~Graphics();
    
    Image obstacleImg_;                             // Image representing obstacles in the environment
    Texture2D texture_img_;                         // Raylib Texture created from obstacleImg
    Model robotModel_;                              // Raylib Model representing a robot. This can be changed.
    Model groundModel_;                             // Model representing the ground plane
    Vector3 groundModelpos_;                        // Ground plane position
    Shader lightShader_;                            // Light shader
    
    Camera3D camera3d = { 0 };                      // Define the camera to look into our 3d world 
    // These represent a set of camera transition frames.
    std::vector<Vector3> camera_positions_{};
    std::vector<Vector3> camera_ups_{};
    std::vector<Vector3> camera_targets_{};
    int camera_idx_ = 0;
    uint32_t camera_clock_=0;    
    bool camera_transition_ = false; 

    // Function to update camera based on mouse and key input
    void update_camera();

};