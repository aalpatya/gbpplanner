# GBPPlanner: Distributing Multirobot Motion Planning with Gaussian Belief Propogation
This code accompanies the 2023 Robotics Automation Letters (RA-L) paper. It was also presented at ICRA 2023.

## Initial Setup
Install Raylib dependencies as mentioned in https://github.com/raysan5/raylib#build-and-installation
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
```shell
./gbpplanner
```

Examples config files are in config directory, and include:
- ```config.json``` (default: robots travel to the opposite sides of a circle in free space)
- ```circle_cluttered.json``` (robots travel to the opposite sides of a circle around some obstacles)
- ```junction.json``` (robots travel in a crossroad junction)
- ```junction_twoway.json``` (robots can travel in both directions, in any road and can turn left, straight, right (red, green, blue) respectively)

Run the simulation:
```shell
./gbpplanner --cfg config/circle_cluttered.json
```

Or create your own config_file.json and run:
```shell
./gbpplanner --cfg config_file.json
```

### During simulation
**Press 'H' to display help and tips!**

Use the mouse wheel to change the camera view (scroll : zoom, drag : pan, shift+drag : rotate)

Press 'spacebar' to transition between camera keyframes which were set in ```src/Graphics.cpp```.

## Play with the code
You may want to create your own formations and scenarios for the robots.

Towards the end of ```src/Simulator.cpp``` there is a function called ```createOrDeleteRobots()```, where you may add your own case.

Alternatively, edit the parameters in the config files and see the effects on the simulations!

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
