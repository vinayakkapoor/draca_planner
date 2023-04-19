# draca_planner

## Overview

Decentralized Deep Reinforcement Learning based motion planning ROS package with collision avoidance. 

The initial neural network is trained using a custom [OCRA](https://gamma.cs.unc.edu/ORCA/) based dataset generated using the [RVO2 library](https://gamma.cs.unc.edu/RVO2/). The neural network is further refined using Reinforcement Learning

## Features

A novel controller for the [mrs uav system](https://github.com/ctu-mrs/mrs_uav_system). Trained PyTorch models can be used to control drone swarms. 

## Prerequisites

Make sure you have 
1. [ROS Melodic / Neotic](http://wiki.ros.org/ROS/Installation)
2. [PyTorch](https://pytorch.org/)
3. [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system)

## Setting up the project

1. Change to the source folder in the workspace using `cd ./workspace/src`
2. Clone the mrs_swarm_core package
3. Change to `decap_exp` branch
4. Clone [draca_planner](https://mrs.felk.cvut.cz/gitlab/vinayak/draca_planner) repo in the ros_packages
5. Build the workspace using `catkin build`

## Running the project 

1. Open */src/mrs_swarm_core/simulation/config/sim_config.yaml* and make sure 
    - uav number is **2**
    - swarm_controller is **draca_planner**
    - exp_world is **"grass_plane"**
2. You may tweak the spawning coordinates of the drones in the *simulation/config/gazebo_config/init_uav_pose/grass_plane.yaml* file. Similarly, the goal coordinates could be changed using the *draca_planner/config/draca_planner.yaml* file
3. Navigate to */src/mrs_swarm_core/simulation*
4. Run `./simulate_swarm.sh -f config/sim_config.yaml`
5. Once the drones takeoff, change to the *run_swm_ctr* window using shift+&#8592; / &#8594; keys
6. Press the &#8593; key to get the autofilled command to run the swarm controller for both the drones. You may use alt+&#8593; / &#8595; to switch between tabs in the window for each drone

## Contributing

## License

## Contact

## Acknowledgments




