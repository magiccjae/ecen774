README.txt

This simulation is part of a project for cooperative target tracking.
The simulation basically includes multiple aerial agents that use gimballed 
cameras to tracking moving ground target

9/19/2014 - Modified by J.Sakamaki
	  - combined a multicopter simulation with geolocation code from ch13 of UAV book
	  - supports 1 moving target and tested with 2 agents. number of agents can be changed with P.num_agents
	  - geolocation is performed using kalman filter, agents are not sharing target information
	  - tested with stationary agents, no noise in camera, fov of cameras overlap to simulation an occlusion