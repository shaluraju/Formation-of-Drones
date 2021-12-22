# Formation of Drones
This repository contains code of the Decentralized Formation Control of Drones which was programmed and tested in Safe Autonomous Systems Lab, Stevens Institue of Technology. This is Hardware Implementation of my [Simulated](https://github.com/shaluraju/Decentralized-Formation-Control-of-UAVs-in-AirSim) Project, several changes were made to the control algorithms to match with the Hardware Components in [SAS](https://saslabstevens.github.io/) Lab

[Ful Video](https://www.youtube.com/watch?v=aNUTGayBirw)

![ezgif com-gif-maker](https://user-images.githubusercontent.com/67613439/147016850-b034dff6-2abe-4957-8277-bb1ee44a64dc.gif)


## Setup and Configuration:
* [Tello Drone](https://www.ryzerobotics.com/tello)
* ROS with Python 3
* [Vicon](https://github.com/ethz-asl/vicon_bridge)
* [Decentralized Formation Control](https://ieeexplore.ieee.org/document/6225196)


#### Bunch of drones are used to form a specified shape(Mostly Characters). Shape Vectors between the drones are predefined and Control Commands are given to minimize the error between the shape vectors. PID gains are used to control the motion of the drones.


### With Lights:



![ezgif com-gif-maker (1)](https://user-images.githubusercontent.com/67613439/147017674-b5056dce-cfd7-4bf1-8b1d-98bd23ea2509.gif)

## Contributor:
* [Mohammad](https://github.com/m-bahrami)
#### Mohammmad is a Phd Student, currently working in [SAS Lab](https://saslabstevens.github.io/), This Project is build on top of his PID Control Code.
#### Thanks to Professor [Hamid](https://faculty.stevens.edu/hjafarne) for guiding me throughout the Project.
