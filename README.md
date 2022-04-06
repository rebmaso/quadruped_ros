
![This is an image](/media/rviz.png)

# RL final project - quadruped robot

This repository demonstrates a control and navigation pipeline for a quadrupedal robot (a React Robotics Dogbot). This project integrates libraries like Towr, Idyntree and Alglib using Ros. The robot task that is shown here consists in looking for a user-specified target QR code among 3 rooms, whose location is known and also assigned by the user. the robot returns to the home location either after finding the right QR code, or after all rooms have been unsuccesfully explored (QR code not found).
The control algorithm implemented in [Controller](popt/src/client/Controller.cpp) is inspired by the whole-body controller in [this paper](https://www.sciencedirect.com/science/article/abs/pii/S0094114X21001701#!). The navigation module in [Planner](popt/src/client/Planner.cpp) is instead based on the [artificial potential fields](https://link.springer.com/chapter/10.1007/978-1-4613-8997-2_29) method. The app that uses these libraries is in [Main](popt/src/client/Main.cpp). To take a look at the demo app in action, please watch [this video](media/presentation.mp4). A written [report](media/rl_report.pdf) is also available in the media section. 

## Setting things up

This project has been tested on Ros Noetic.

Install all external dependencies (Ros Noetic, Idyntree, Zbar, Ifopt, and Hector Mapping):

`` sudo apt-get install ros-noetic-desktop-full zbar-tools ros-noetic-ifopt ros-noetic-hector-slam``

Follow [these instructions](https://github.com/robotology/idyntree#installation) to install Idyntree with Conda, or build it from source.

This project also uses Alglib and Towr, but you don't need to install them. The headers are in the [include](popt/include) folder.

Clone the repo in your current ros workspace and 

`` source ~/<your_ws>/devel/setup.bash ``

Build the workspace with the release flag:

`` catkin_make -DCMAKE_BUILD_TYPE=Release ``

## Launching the simulator

Spawn the world and the dogbot model in gazebo:

`` roslaunch dogbot_gazebo dogbot_sim.launch <gui:=false> <rviz:=true>``

This launch file also runs rviz with a custom lightweight config. You can optionally use the gazebo gui using the input args as shown.

## Running the app (control + navigation)

Launch the node:

`` roslaunch popt main.launch <target_id:="id_2">``

The ``main`` app unpauses gazebo and runs both the control and the navigation pipelines. This launch file also starts the ``hector_mapping`` node, which uses lidar scans to build a globally consistent 2D occupancy grid map of the environment, along with a 2D base link pose estimate. The map is used by the navigation algorithm. You can optionally pass the target qr code as an argument, as shown.

