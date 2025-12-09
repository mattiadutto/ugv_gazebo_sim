# AgileX Product Gazebo Fortress Simulate
This repository is working with ROS2 Humble and Gazebo Fortress.

## Install the Gazebo software

Gazebo is  a simulator. Gazebo simulates multiple robots in a 3D environment, with extensive dynamic interaction between objects.

[Gazebo Fortress installation guide](https://gazebosim.org/docs/fortress/install/).

Download and install gazebo you can go to the website :https://gazebosim.org/docs/fortress/install/

------

## Current support Gazebo simulation product list

| Product name     | support status |
| :--------------- | -------------- |
| BUNKER           | √              |
| HUNTER 1.0       |                |
| HUNTER 2.0       |                |
| HUNTER SE        |                |
| LIMO             |                |
| RANGER MINI      |                |
| SCOUT 1.0        |                |
| SCOUT 2.0        |                |
| SCOUT MINI       | √              |
| SCOUT MINI(OMNI) |                |
| TRACER           |                |


## About usage

1) Clone the current repositories to your own workspace
2) Go to you use product
3) Each independent chassis product has its own independent instructions in the corresponding file directory

------
A few notes:
> The idea is to extend this repository to support the Bunker robot in simulation, and later, when I will move to a newer version of ROS2 and Gazebo update for the already supported robot.

> Feel free to write to me or to open a pull request if you want to expand the support. 