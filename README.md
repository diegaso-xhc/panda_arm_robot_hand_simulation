# Simulation of a 7DOF-Robot + 7DOF-Robot-Hand system

## Overview of the repository
<div align="justify">
The usage of robotic hands for grasping is one of the main disciplines being explored within the robotics field in recent years. State of the art robot hand designs and control algorithms have not yet reached a performance level comparable to human hands. Part of the reason for this is that conducting real-world experiments to test state-of-the-art algorithms is a convoluted procedure. This repository aims to providing the robotics community with a robot-arm-hand simulation environment in which diverse control strategies can be tested. In our most recent work (currently in publication) we used a similar setup in real-world experiments. Once the journal article is pusblished, we will release the code for transfering of motions to the real world.
<br />
<br />
This repository contains an easy-to-use c++ interface with Mujoco, where a fully customized model for a panda robot arm + seed robotics RH8D hand are built. The environment has been designed for good visualizations, so users can directly showcase their applications, without spending extra time into good graphics.

<br />
<br />

<p align="center">
   <img src="/Visualizations/Simtoreal.png" width="700" />
</p>

<br />

## Understanding repository

The repository was developed using the following software version:

```
- Ubuntu 20.04.5 LTS
- g++ (Ubuntu 9.4.0-1ubuntu1~20.04.2)
- c++ (Ubuntu 9.4.0-1ubuntu1~20.04.2)
- Mujoco 2.3.0
```
To run the repository, please create a folder called "Projects" within your mujoco folder. Then clone the repo within this folder. Following this step, the user can directly modify controllers or robot behaviors by editing the main_panda_RH8D.cpp file. Then simply run:

```
./run_code_ubuntu
```

Then a window will be launched where the controllers of your main file will be simulated. 
<br />

## Contributions

The contributions of this repository can be summarized as follows:

```
- A ready to use panda-arm-robot-hand simulation.
- A simple controller framework so users can implement their controllers.
- In the next couple of months (once our work is published) we will add the algorithms required for motions in real-world.
```

## License

Developed by Diego Hidalgo C. (2023). This repository is intended for research purposes only. If you wish to use any parts of the provided code for commercial purposes, please contact the author at hidalgocdiego@gmail.com.
