## Table of contents
* [General info](#general-info)
* [Simulation](#technologies)
* [Real Drone Setup](#setup)

## General info
This master thesis on "Autonomous Landing of an Unmanned Aerial Vehicle on a Moving Platform" was implemented in Simulation, using ROS Kinetic and Gazebo 7.
	
# Simulation
To run this simulation, first follow the installation steps on the following repositories:
* durable_ist/Multi_Robot_Simulation: https://github.com/durable-ist/Multi_Robot_Simulation.git
*  UbiquityRobotics/fiducials: https://github.com/UbiquityRobotics/fiducials.git

To launch the marker pattern on top of the UGV, the following needs to be done:
* Replace the folder multi_jackal->multi_jackal_description ; with the one with the same name in the Simulation folder of this GitHub

To launch the aruco detect node with the correct parameters, the following needs to be done:
* Replace the file fiducials -> aruco_detect -> launch -> aruco_detect.launch, by the file with the same name in the Simulation folder of this GitHub

## Launch Instructions
To run this project, insert the following commands on three different terminals:

Terminal 1:
```
$ roscore
```

Terminal 2:
```
$ roslaunch simulation.launch
```

Terminal 3:
```
$ sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 --console -I0 -L Evora
```

To change the ardupilot to Guided mode run in a separate terminal the following command:

```
$ rosrun mavros mavsys mode -c 4
```

When the console pop-up displays "arm good", insert the following commands in the 3rd Terminal to arm the drone and takeoff to a certain height:

```
$ arm throttle
$ takeoff 6
```

On a separate terminal run the python code with the proposed implementation:
```
$ python fsm.py
```

	
# Real Drone Setup
The aruco detectipn algorithm has already been installed in the real drone in this situation, along with the remaining necessary packages, but if not, the aruco detect package needs to be installed in the drone, the same way it is installed in simulation, and the file to be replaced is called aruco_detect3.launch located in the Real folder.

## Launch Instructions
To run this project, first all terminals need to be connected to the drone through ssh:
```
$ ssh apsync@10.0.1.128 -X
```
* Password: apsync

Then, run the following launch file to run thr mavros node, the realsense node, the aruco detection node, and the necessary transforms:
```
$ roslaunch transforms.launch
```

On a separate terminal run the python code with the proposed implementation:
```
$ python fsm.py
```