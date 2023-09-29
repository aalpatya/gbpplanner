#define RLIGHTS_IMPLEMENTATION // needed to be defined once for the lights shader
#include <iostream>
#include <Utils.h>

#include <DArgs.h>

#include <Globals.h>
#include <Simulator.h>

Globals globals;
int main(int argc, char *argv[]){
    
    srand((int)globals.SEED);                                   // Initialise random seed   
    DArgs::DArgs dargs(argc, argv);                             // Parse config file argument --cfg <file.json>
    if (globals.parse_global_args(dargs)) return EXIT_FAILURE;  
    
    Simulator* sim = new Simulator();       // Initialise the simulator
    globals.RUN = true;
    while (globals.RUN){

        sim->eventHandler();                // Capture keypresses or mouse events             
        sim->createOrDeleteRobots();        
        sim->timestep();
        sim->draw();

    }

    delete sim;

    return 0;
}    
