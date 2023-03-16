# Robots

In this repository, the scripts for some preliminary path planning algorithms such as bug1 and APF along with a multi-agent swarm aggregation simulation  and a SLAM algorithm can be found. 

## Table of Contents

- [About](#about)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [bug1](#bug1)
- [Localization Using Kalman Filter](#localization-using-kalman-filter)
- [APF](#apf)
- [Swarm Aggregation](#swarm-aggregation)
- [Contributing](#contributing)
- [License](#license)

## About

This project contains some basic path planning algorithm which helps in understanding the implmentation aspects of these algorithms. The script of kalman filter using trilateration is also provided here for anyone intereseted in SLAM. An introductory implementation for aggregation of swarm of agents is also provided in this project.

## Getting Started

In order to run the scripts:

- Ubuntu 20.04 LTS (recommended)
- Any test editor (preferably VS Code)
- ROS Noetic
- turtlebot3 burger package

Some libraries used were:

- rospy
- Numpy
- Matplotlib
- Math

## Usage

1. Clone this repository
```terminal
git clone https://github.com/swe-sys/robots.git
```
2. Run the launch script:
```terminal
roslaunch robots launch_script_name.launch
```
3. Execute python scripts of the same folder:
```terminal
rosrun robots script_name.py
```
4. Add star to this repo if you like it :wink:.

## Bug1
In this project, the robot detects the obstacle using its range sensor. When the robots detects an obstacle it first circumnavigate the obstacle, then leave from the shortest distance point to goal to reach its goal position, if path exists `bug1.py`.

## Localization using Kalman Filter
In this project, three vitual obstacles are used and their positions consists of the trilateration data. The robot is desired to form a circular trajectory where kalman filter `kalman.py` estimates the position of the robot using the measurement data (trilateration data) `trilateration.py`.

## APF
In this project, robot is provided an attractive potential to real the goal position and repulsive potential to avoid the obstacles. Then using gradient descent algorithm the way points is calculated `APF_plain.py`. For the robot to track the calculated waypoints a controller is used `PID_ control.py`.

## Swarm Aggregation
In this project, 5 robots are provided different decentralized control laws when the robot is in free state or engaged state. 

## Contributing

Any contribution is welcome!! 

## License

BSD 3-Clause License
