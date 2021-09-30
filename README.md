## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
* [Setup](#setup)

## General info
This master thesis on "Autonomous Landing of an Unmanned Aerial Vehicle on a Moving Platform" was implemented in Simulation, using ROS Kinetic and Gazebo 7.
	
## Simulation
To run this simulation, first follow the installation steps on the following repositories:
* durable_ist/Multi_Robot_Simulation: https://github.com/durable-ist/Multi_Robot_Simulation.git
*  UbiquityRobotics/fiducials: https://github.com/UbiquityRobotics/fiducials.git

To launch the marker pattern on top of the UGV, the following needs to be done:
* Replace the folder multi_jackal->multi_jackal_description ; with the one with the same name in the Simulation folder of this GitHub

To launch the aruco detect node with the correct parameters, the following needs to be done:
* Replace the file fiducials -> aruco_detect -> launch -> aruco_detect.launch, by the file with the same name in the Simulation folder of this GitHub

# Launch Instructions
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

	
## Real Drone Setup
To run this project, install it locally using npm:

```
$ cd ../lorem
$ npm install
$ npm start
```