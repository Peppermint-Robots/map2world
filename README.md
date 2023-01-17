# map2world
ROS2 Port of map2gazebo: A package for creating Gazebo/Ignition environments from 2D Maps

map2gazebo Project URL: https://github.com/shilohc/map2gazebo

## Pre-Requisites
- ROS2 Humble (tested with the Debian packages)
- Python3 Dependencies
    - opencv-python
    - numpy
    - trimesh
    - matplotlib
    - pycollada

## Installation
- Make sure all pre-requisites are installed on your system first.
- Create a new folder. Let's say it's map2world.
- Create a src folder inside it.
- Open a fresh terminal. `cd` into map2world (or whatever name you had given).
- Execute `colcon build` in the terminal.

## Usage
- Make sure the installation is properly completed
- Have a map server publishing the map to a topic. (One way to do is to run your Robotics Stack locally.)
- Open a fresh terminal, `cd` into the directory where map2world was built, and source it (`source install/setup.bash`)
- Run map2world: `ros2 run map2world map2world --ros-args -p map_topic:={map_topic} -p mesh_type:={mesh_type} -p export_dir:={export_dir} -p occupied_threshold:={occupied_threshold} -p box_height:={box_height}`
    - Parameters Reference:
        - map_topic (Optional): Map Topic to which the map is being published. Default Value: _map_
        - mesh_type (Optional): Whether to export as a .dae file or .stl file. Accepted Values: _.dae_, _.stl_ Default Value: _.dae_
        - export_dir (Optional): Folder path to save the file to. Default Value: Active user's home directory.
        - occupied_threshold (Optional): Minimum threshold for OpenCV to get the occupied regions of the map. Default Value: _1_
        - box_height (Optional): Height of the occupied voxel from ground in the 3D mesh. Default Value: _2.0_
- Once the conversion is successfully done, the map2world node will shut down with a message on the terminal.