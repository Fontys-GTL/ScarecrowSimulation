# ScarecrowSimulation
Repository for sharing the simulation created during the scarecrow project
In case of any more questions, the subfolders contain more information on how to install everything. The setup was tested in two environments and worked in both of them. However,
PX4 can be a bit tricky with installations. In that case. look into the PX4-ROS2-Gazebo folder on how to install it properly for this project.


# Simulation
This project repository contains 3 main parts:
- Gazebo-terrain-generator: This project is used to generate the terrain for the simulation. It uses a heightmap to create a realistic terrain in Gazebo.
- PX4-Autopilot: The PX4-Autopilot is used to control the drone in the simulation. It is a popular open-source autopilot software for drones. All world files and models are stored in this repository. They can be found under PX4-Autopilot/Tools/simulation/gz/worlds and PX4-Autopilot/Tools/simulation/gz/models.
- ws_offboard_control: This is the ROS2 workspace that contains an example offboard_control node. This node is used to control the drone in the simulation using ROS2. It is a simple example that can be used as a starting point for more complex control algorithms. Currently it arms the drone and sends precoded waypoints to the drone. The node can be found under ws_offboard_control/src/px4_ros_com/src/examples/offboard_py/offboard_control.py. 

# How to run the terrain generator
To install the terrain generator, execute the following commands:
```python3 -m venv terrain_generator
source terrain_generator/bin/activate
pip install -r requirements.txt
```


To run the terrain generator, follow these steps:
1. Navigate to the Gazebo-terrain-generator folder in the terminal.
2. Run the following command to start the terrain generator:
```
source terrain_generator/bin/activate
python scripts/server.py
```
3. Open a web browser and navigate to http://localhost:8080 to access the terrain generator interface.
4. Use the interface to create your desired terrain and click on the "Generate" button to create the heightmap and gazebo world (.sdf).
5. Output is stored in the output folder by default. You can change the output path by setting GAZEBO_MODEL_PATH and GAZEBO_WORLD_PATH environment variables to the desired output folders.
# How to run the simulation