## Autonomous Butler 
This is a package for autonomous butler which I have been building just for fun.
This repo contains a small differential bot. Not a full butler shaped because, this repo is only for testing out algorithms, the diff bot will be replaced by a butler bot later on. 

Steps to run this:- 
 - Clone:- 
```bash
   git clone https://github.com/Bhajneet-Singh-Bedi/autonomous_butler.git src
```
 - Build:- 
```bash
   colcon build
```
 - To run:- 
```bash
   ros2 launch autonomous_butler mission.launch.py
```

NOTE:-
If in case, it gives error for .obj file not found for model.\
Then run:-
```bash
   export GZ_SIM_RESOURCE_PATH=`pwd`/src:$GZ_SIM_RESOURCE_PATH
```
Run this while in workspace folder.\
Peace Out ✌️