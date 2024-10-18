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
    - shapely

## Installation
- Make sure all pre-requisites are installed on your system first.
- Create a new folder. Let's say it's map2world.
- Create a src folder inside it. 
- Open a fresh terminal. `cd` into map2world/src. Clone this repository.
- Execute `cd ..` in the same terminal to move a folder up to map2world.
- Execute `colcon build` in the terminal.

## Usage
- Make sure the installation is properly completed
- Rember to change the path for template folder (variable: self.template_path).
- Do verify the location where the model is being saved (variable: package_path). By default it finds the share directory of package_name given.
- Have a map server publishing the map to a topic. (One way to do is to run your Robotics Stack locally.)
- Open a fresh terminal, `cd` into the directory where map2world was built, and source it (`source install/setup.bash`)
- Run map2world: `ros2 run map2world map2world --ros-args -p map_topic:={map_topic} -p mesh_type:={mesh_type} -p occupied_threshold:={occupied_threshold} -p box_height:={box_height} -p package_name:= {package_name} -p model_name:= {model_name} -p img_path:={img_path} -p map_mode:={map_mode} -p red:={red} -p green:={green} -p blue:={blue}`
    - Parameters Reference:
        - map_topic (Optional): Map Topic to which the map is being published. Default Value: _map_
        - mesh_type (Optional): Whether to export as a .dae file or .stl file. Accepted Values: _.dae_, _.stl_ Default Value: _.dae_
        - occupied_threshold (Optional): Minimum threshold for OpenCV to get the occupied regions of the map. Default Value: _1_
        - box_height (Optional): Height of the occupied voxel from ground in the 3D mesh. Default Value: _2.0_
        - package_name (Optional): The package where the mesh file and model files will be stored. Default Value: _sim_models_ppmt_
        - model_name (Optional): The name by which user wants to save the model. Default Value: _new_model_
        - img_path (Optional): The location of image where the path image is stored. Default Value: _"-1"_
        - map_mode (Optional): The mapping mode. Default Value: _clean_
        - red (Optional): Path colour user requires in map. Default Value: _255_
        - green (Optional): Path colour user requires in map. Default Value: _0_
        - blue (Optional): Path colour user requires in map. Default Value: _0_
- Once the conversion is successfully done, the map2world node will shut down with a message on the terminal.
