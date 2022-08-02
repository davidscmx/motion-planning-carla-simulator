# Purpose

This repository contains the starter code to launch in the SDC Planning course workspace.
Together with the necessary infrastructure to run with Docker container and Carla.


I will prepare the Docker container to make running the trajectory planning demo as smoothly
as possible.

Needed
- sudo apt-get install libboost-all-dev
- sudo apt-get install libblas-dev liblapack-dev

## Carla Docker Container Instructions
https://carla.readthedocs.io/en/latest/build_docker/

Requisites:

You will need to have installed:

Docker: Follow the installation instructions here.
NVIDIA Container Toolkit: The NVIDIA Container Toolkit is a library and toolset that exposes NVIDIA graphics devices to Linux containers. It is designed specifically for Linux containers running on Linux host systems or within Linux distributions under version 2 of the Windows Subsystem for Linux. Install the nvidia-docker2 package by following the instructions here.


Running Carla with Docker:
```bash
docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:latest /bin/bash ./CarlaUE4.sh
```