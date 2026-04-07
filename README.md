# ScarecrowSimulation
Repository for sharing the simulation created during the scarecrow project
In case of any more questions, the subfolders contain more information on how to install everything. The setup was tested in two environments and worked in both of them. However,
PX4 can be a bit tricky with installations. In that case. look into the PX4-ROS2-Gazebo folder on how to install it properly for this project.

Used in this project were ROS2 Humble and Gazebo Harmonic 8.11.0. This worked fine in some cases, but an update broke parts of the ROS2-Gazebo communication. Most data is still being sent, but images are not bridged correctly. This might be fixed with a newer version of both ROS2 and Gazebo. If you want to use the simulation, I would recommend using ROS2 Humble and Gazebo Harmonic 8.11.0, as this is the version that was used during development and testing.

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
For now, we'll assume that you have installed PX4 and MAVROS properly. If you haven't, please refer to the PX4-Autopilot and PX4-ROS2-Gazebo folders for instructions on how to install it.

To run the simulation in its entirety, you have to open multiple terminals. Start each terminal from the root of the repository and execute the following commands in each terminal:
Terminal 1:
```
cd ~/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888
```
Terminal 2:
You will need to export the path to the resources used in the simulation. This will change based on where you have stored the repository. For example, if you have stored the repository in /home/gtl/Development/ScarecrowSimulation, you will need to export the following paths:
```
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/gtl/Development/ScarecrowSimulation/PX4-ROS2-Gazebo-YOLOv8/models:/home/gtl/Development/ScarecrowSimulation/gazebo_terrain_generator/output/gazebo_terrain
```
Then, navigate to the PX4-Autopilot folder and run the following command to start the simulation:
```
source px4-venv/bin/activate
cd PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL_POSE="0.0, 0.0,0.2,0.00,0,-0.7" PX4_GZ_WORLD=1775128397976 PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4
```
This will launch the PX4 stack and Gazebo with a sample generated terrain. It loads up the x500_depth_model, this can be changed by specifying a different drone. You can also specify a different world: this world has to be in the PX4-Autopilot/Tools/simulation/gz/worlds folder. 

Terminal 3:
In the third terminal, navigate to the ws_offboard_control folder and run the following command to start the offboard control node:
```
cd ws_offboard_control
source install/setup.bash
ros2 run px4_ros_com offboard_control.py
```
This will start the offboard control node, which will arm the drone and send precoded waypoints to the drone. You should see the drone take off and fly to the specified waypoints in the Gazebo simulation. You can modify the waypoints in the offboard_control.py file to change the flight path of the drone.