/**************************************************************************************/
// Copyright (c) 2023 Aalok Patwardhan (a.patwardhan21@imperial.ac.uk)
// This code is licensed under MIT license (see LICENSE for details)
/**************************************************************************************/
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <Utils.h>
#include <Globals.h>
extern Globals globals;

/**************************************************************************************/
// Draw the Time and FPS, as well as the help box
/**************************************************************************************/
void draw_info(uint32_t time_cnt){
    static std::map<MODES_LIST, const char*> MODES_MAP = {
        {SimNone,""},
        {Timestep,"Timestep"},
        {Iterate,"Synchronous Iteration"},
        {Help, "Help"},
    };

    int info_box_h = 100, info_box_w = 180;;
    int info_box_x = 10, info_box_y = globals.SCREEN_SZ-40-info_box_h;

    DrawRectangle(0, globals.SCREEN_SZ-30, globals.SCREEN_SZ, 30, RAYWHITE);
    DrawText(TextFormat("Time: %.1f s", time_cnt*globals.TIMESTEP), 5, globals.SCREEN_SZ-20, 20, DARKGREEN);
    DrawText(TextFormat("Press H for Help"), globals.SCREEN_SZ/2-80, globals.SCREEN_SZ-20, 20, DARKGREEN);
    DrawFPS(globals.SCREEN_SZ-80, globals.SCREEN_SZ-20);

    if (globals.SIM_MODE==Help){
        int info_box_h = 500, info_box_w = 500;
        int info_box_x = globals.SCREEN_SZ/2 - info_box_w/2, info_box_y = globals.SCREEN_SZ/2 - info_box_h/2;
        DrawRectangle( info_box_x, info_box_y, info_box_w, info_box_h, Fade(SKYBLUE, 0.5f));
        DrawRectangleLines( info_box_x, info_box_y, info_box_w, info_box_h, BLACK);

        int offset = 40;
        std::vector<std::string> texts{
            "Esc : \t\t\t\t Exit Simulation",
            "H : \t\t\t\t\t\t Close Help",
            "SPACE : \t Camera Transition",
            "ENTER : \t Run/Pause Simulation",
            "P : \t\t\t\t\t\t Toggle Planned paths",
            "R : \t\t\t\t\t\t Toggle Connected Robots",
            "W : \t\t\t\t\t\t Toggle Waypoints",
            ""         ,
            "Mouse Wheel Scroll : Zoom",
            "Mouse Wheel Drag : Pan",
            "Mouse Wheel Drag + SHIFT : Rotate",
        };

        for (int t=0; t<texts.size(); t++){
            DrawText(texts[t].c_str(), info_box_x + 10, info_box_y + (t+1)*offset, 20, BLACK);
        }
    }
}


