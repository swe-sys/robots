# Robots

In this repository, the scripts for some preliminary path planning algorithms such as bug1 and APF along with a multi-agent swarm aggregation simulation  and a SLAM algorithm can be found. 

## Table of Contents

- [About](#about)
- [Getting Started](#getting-started)
- [Usage](#usage)
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

## Contributing

Any contribution is welcome!! 

## License

BSD 3-Clause License
