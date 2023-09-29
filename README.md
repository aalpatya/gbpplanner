# GBPPlanner: Distributing Multirobot Motion Planning with Gaussian Belief Propogation
This code accompanies the 2023 Robotics Automation Letters (RA-L) paper. It was also presented at ICRA 2023.

## Initial Setup
Install Raylib dependencies as mentioned in https://github.com/raysan5/raylib.
This will be platform dependent

Clone the repository *with the submodule dependencies:*
```shell
git clone https://github.com/aalpatya/gbpplanner.git --recurse-submodules
```
Use CMAKE to set up the build environment and then run 'make':
```shell
mkdir build
cd build
cmake ..
make
```

## Run examples
Make any changes to the simulations you want in config/config.json and then run:
```./gbpplanner```

Examples config files are in config directory, and include:
- circle_cluttered (robots travel to the opposite sides around some obstacles)
- junction (robots travel in a crossroad junction)
- junction_twoway (robots can travel in both directions, in any road and can turn left, straight, right (red, green, blue) respectively)
```./gbpplanner --cfg config/circle_cluttered.json```

Or create your own config_file.json and run:
```./gbpplanner --cfg config_file.json```

Press 'H' to display help and tips!

## Play with the code
You may want to create your own formations and scenarios for the robots.
Turn your attention to src/Simulator.cpp
Towards the end of that file, there is a function called createOrDeleteRobots(), where you may add your own case.


## Cite us
```
@ARTICLE{gbpplanner,
        author={Patwardhan, Aalok and Murai, Riku and Davison, Andrew J.},
        journal={IEEE Robotics and Automation Letters}, 
        title={Distributing Collaborative Multi-Robot Planning With Gaussian Belief Propagation}, 
        year={2023},
        volume={8},
        number={2},
        pages={552-559},
        doi={10.1109/LRA.2022.3227858}}
```
