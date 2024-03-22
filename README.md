# Third Year Project

Swarm robotics dissertation project

The protection of a VIP agent with a convoy swarm

## Dependencies

Stage4 Robotics Simulator, available here: https://github.com/rtv/Stage/

This project uses stage as a library, through the libstage plugin for C++

## Languages Used

- Python for testing and running automation
- C++ for main logic

Note: C++11 required, it is enforced in the compilation script, some features not available in previous C++ versions are used.

Note++: Compilation is set up using clang in makefile as I use macos, however you can change this very easily by editing makefiles!

## Directories

- common: contains common files used between all of the different algorithms
- boids: this directory contains the lowest level boids algorithm
- boids_follow_circle: circle-based approach to protecting the VIP agent
- boids_follow_classic: this directory contains the boids algorithm that follows the VIP agent using a classic boids weighted algorithm
- testing: contains scripts for testing and evaluating the data
