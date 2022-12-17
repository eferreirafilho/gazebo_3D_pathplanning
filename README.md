# 
# UAV simulation in gazebo

Uses the UAV repository and its dependencies:
http://github.com/RAFALAMAO/hector-quadrotor-noetic

# To use the path planning: 
Add folders found in this repo replacing original http://github.com/RAFALAMAO/hector-quadrotor-noetic files when asked

# To launch the UAV and gazebo world:
roslaunch hector_quadrotor_demo outdoor_church.launch

# To build the PRM from pcd:

run file load_pcd.py
  - generates milestones
  - saves point_cloud_prm.npy
  - saves milestone_roadmap_prm.pickle


run file construct_prm.py
  - checks collisions and generate PRM
  - save prm_graph.pickle

# To input a goal:
modify /params/goal.yaml 

# To launch the path planning algorithm:
roslaunch hector_quadrotor_demo prm.launch
