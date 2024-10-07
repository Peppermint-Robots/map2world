import cv2
import numpy as np
import trimesh
from matplotlib.tri import Triangulation
from pathlib import Path
import os
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid
from ament_index_python.packages import get_package_share_directory
from shapely.geometry import Polygon

# Make sure to have pycollada pip package installed for the DAE build to work.


class MapConverter(Node):
    def __init__(self):
        super().__init__("map2world")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("mesh_type", "dae")
        self.declare_parameter("occupied_threshold", 1)
        self.declare_parameter("box_height", 2.0)
        self.declare_parameter("package_name", "sim_models_ppmt")
        self.declare_parameter("model_name", "new_model")
        self.declare_parameter("red", 0)
        self.declare_parameter("green", 0)
        self.declare_parameter("blue", 0)
        

        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.mesh_type = (
            self.get_parameter("mesh_type").get_parameter_value().string_value
        )
        self.threshold = (
            self.get_parameter("occupied_threshold").get_parameter_value().double_value
        )
        self.height = (
            self.get_parameter("box_height").get_parameter_value().double_value
        )
        self.package_name = (
            self.get_parameter("package_name").get_parameter_value().string_value
        )
        self.model_name = (
            self.get_parameter("model_name").get_parameter_value().string_value
        )

        self.red = self.get_parameter("red").get_parameter_value().integer_value
        self.green = self.get_parameter("green").get_parameter_value().integer_value
        self.blue = self.get_parameter("blue").get_parameter_value().integer_value

        map_sub_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.test_map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, qos_profile=map_sub_qos
        )
        # self.export_dir = str(os.getcwd())
        # Probably there's some way to get trimesh logs to point to ROS
        # logs, but I don't know it.  Uncomment the below if something
        # goes wrong with trimesh to get the logs to print to stdout.
        # trimesh.util.attach_to_log()

    def map_callback(self, map_msg):
        """
        Processes a 2D occupancy grid map and converts it into a 3D mesh model.

        This function subscribes to a 2D occupancy grid map, processes the data to
        identify occupied regions, and generates a 3D mesh model from the contours
        of those regions. The resulting mesh can be exported in STL or DAE format
        based on the specified `mesh_type`.

        Args:
            map_msg (nav_msgs.msg.OccupancyGrid): ROS OccupancyGrid message containing
            the 2D map data.
        """
        self.get_logger().info("Lets trun 2D map into 3D maps!.")
        model_name = self.model_name
        package_path = str(get_package_share_directory(self.package_name))
        self.create_project_structure(
            model_name=model_name, mesh_type=self.mesh_type, package_path=package_path
        )
        self.get_logger().info("Received map. Processing.")
        map_dims = (map_msg.info.height, map_msg.info.width)
        map_array = np.array(map_msg.data).reshape(map_dims)

        # set all -1 (unknown) values to 0 (unoccupied)
        map_array[map_array < 0] = 0
        contours = self.get_occupied_regions(map_array)
        meshes = self.contour_to_mesh(contours, map_msg.info)

        mesh_wall = trimesh.util.concatenate(meshes[0])
        mesh_line = trimesh.util.concatenate(meshes[1])
        # Export as STL or DAE
        mesh_type = self.mesh_type

        if mesh_type == "stl":
            with open(
                package_path + f"/models/{model_name}/meshes/{model_name}_wall.stl",
                "wb",
                # export_dir + f"/{model_name}/{model_name}/meshes/{model_name}.stl", "wb"
            ) as f:
                mesh_wall.export(f, "stl")
            with open(
                package_path + f"/models/{model_name}/meshes/{model_name}_line.stl",
                "wb",
                # export_dir + f"/{model_name}/{model_name}/meshes/{model_name}.stl", "wb"
            ) as f:
                mesh_line.export(f, "stl")
            self.get_logger().info("Exported STL. Shutting down this node now.")
        if mesh_type == "dae":
            with open(
                package_path + f"/models/{model_name}/meshes/{model_name}_wall.dae", "wb"
            ) as f:
                f.write(trimesh.exchange.dae.export_collada(mesh_wall))
            with open(
                package_path + f"/models/{model_name}/meshes/{model_name}_line.dae", "wb"
            ) as f:
                f.write(trimesh.exchange.dae.export_collada(mesh_line))
            self.get_logger().info("Exported DAE. Shutting down this node now.")

    def get_occupied_regions(self, map_array):
        """
        Identifies occupied regions in a 2D map using contour detection.

        This function processes the input 2D occupancy grid map and perform contour
        detection, to identify regions that are considered occupied. The detected
        contours represent the boundaries of the occupied areas in the map.

        Args:
            map_array (numpy.ndarray): A 2D numpy array representing the occupancy grid map.
            The map contains occupancy values where -1 indicates unknown, 0 indicates
            unoccupied, and positive values indicate occupied regions.
            model_name (str): The name of the model, used for saving contour images
            for debugging purposes.

        Returns:
            list: A list of contours where each contour represents an occupied region
            in the 2D map. Each contour is a numpy array of points that define the boundary
            of the region.
        """
        map_array = map_array.astype(np.uint8)
        contour_img = np.zeros_like(map_array)
        output = np.zeros_like(map_array)
        output_line = np.zeros_like(map_array) 
        # if self.mode=="line":
            # map_array = cv2.blur(map_array, (5,5))
        contours, hierarchy = cv2.findContours(
            map_array, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
        )
        # Draw the inner path contours
        cv2.drawContours(contour_img, contours, -1, (255, 255, 255), thickness=1)
        
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(map_array, connectivity=8)
        
        for j in range(1, num_labels):
            cv2.putText(contour_img, str(j), (int(centroids[j][0]),int(centroids[j][1])) , cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255), 1)
        
        cv2.imshow("Component Ids", contour_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        path_id = input("Enter which path are required (give it in space seperated format):")
        path_ids = path_id.split(' ')
        
        for label in range(1, num_labels):
            if str(label) not in path_ids:
                component_mask = (labels == label).astype("uint8") * 255
                output = cv2.bitwise_or(output, component_mask)
            else:
                component_mask = (labels == label).astype("uint8") * 255
                output_line = cv2.bitwise_or(output_line, component_mask)
                
        
        contours_wall, hierarchy_wall = cv2.findContours(
            output, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
        )
        
        hierarchy_wall = hierarchy_wall[0]
        corner_idxs_wall = [i for i in range(len(contours_wall)) if hierarchy_wall[i][3] == -1]     
        
        contours_line, hierarchy_line = cv2.findContours(
            output_line, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
        )
        
        if len(contours_line) !=0: 
            hierarchy_line = hierarchy_line[0]
            corner_idxs_line = [i for i in range(len(contours_line)) if hierarchy_line[i][3] == -1]
            return [[contours_wall[i] for i in corner_idxs_wall], [contours_line[i] for i in corner_idxs_line]]
        else:
            return [[contours_wall[i] for i in corner_idxs_wall],""]
            
    def contour_to_mesh(self, contour, metadata):
        """
        Converts 2D contours into a 3D mesh by extruding the contour lines.

        This function takes in a set of 2D contour points and generates a 3D mesh by
        extruding the contour lines vertically. The height of the extrusion is
        determined by a predefined valuezz, and the mesh is constructed using the
        `trimesh` library.

        Args:
            contour (list): A list of contours, where each contour is an array of
            2D points representing the boundary of an occupied region in the map.
            metadata (object): Metadata about the map, which contains information
            such as resolution and origin, used to convert map coordinates to
            actual world coordinates.

        Returns:
            trimesh.Trimesh: A 3D mesh generated by extruding the 2D contour lines.
            The mesh consists of vertices and faces that define the geometry.
        """
        
        meshes_line = []
        meshes_wall = []
       
        for point in contour[0]:
            new_point_array = []
            for points in point:
                x, y = points[0]
                new_point = self.coords_to_loc((x, y), metadata)
                new_point_array.append(new_point)
            height = self.height
            pixel_size = metadata.resolution
            
            for p in new_point_array:
                x, y = p
                # Create a small square around the pixel
                pixel_polygon = Polygon([
                    (x, y),  
                    (x + pixel_size, y), 
                    (x + pixel_size, y + pixel_size),  
                    (x, y + pixel_size)  
                ])
               
                mesh_wall = trimesh.creation.extrude_polygon(pixel_polygon, height)
                meshes_wall.append(mesh_wall)
                
        if contour[1] != "":        
            for point in contour[1]:
                new_point_array = []
                for points in point:
                    x, y = points[0]
                    new_point = self.coords_to_loc((x, y), metadata)
                    new_point_array.append(new_point)

                height = 0.001
                pixel_size = metadata.resolution  
                
                for p in new_point_array:
                    x, y = p
                    # Create a small square around the pixel
                    pixel_polygon = Polygon([
                        (x, y),  
                        (x + pixel_size, y), 
                        (x + pixel_size, y + pixel_size),
                        (x, y + pixel_size)  
                    ])
                   
                    mesh_line = trimesh.creation.extrude_polygon(pixel_polygon, height)
                    face_colors = [self.red,self.green,self.blue]
                    mesh_line.visual.face_colors = face_colors
                    meshes_line.append(mesh_line)
        
        mesh = trimesh.util.concatenate(meshes_wall, meshes_line)
        mesh.show()
        mesh.remove_duplicate_faces()

        return [meshes_wall, meshes_line]

    def coords_to_loc(self, coords, metadata):
        """
        Converts 2D map coordinates to real-world locations using map metadata.

        This function transforms the given 2D coordinates from the map frame
        into real-world locations by applying the map's resolution and origin
        information from the metadata. The transformation assumes that the origin
        has no rotation and is at a z-height of zero.

        Args:
            coords (tuple): A tuple (x, y) representing the 2D coordinates in the
            map frame.
            metadata (object): An object containing the map metadata, including
            resolution and origin. This metadata is used to scale and shift
            the coordinates to the real-world frame.

        Returns:
            list: A list [loc_x, loc_y] representing the real-world coordinates
            corresponding to the input map coordinates.
        """
        x, y = coords
        loc_x = x * metadata.resolution + metadata.origin.position.x
        loc_y = y * metadata.resolution + metadata.origin.position.y
        # TODO: transform (x*res, y*res, 0.0) by Pose map_metadata.origin
        # instead of assuming origin is at z=0 with no rotation wrt map frame
        return (loc_x, loc_y)

    def create_project_structure(self, model_name, mesh_type, package_path):
        """
        Creates a directory structure for a Gazebo model and world simulation.

        This function generates the necessary folders and files for a Gazebo
        simulation, including directories for model, world, and mesh data. It also
        creates placeholder configuration files (`model.config`, `model.sdf`, and
        `world.sdf`) with predefined content based on the provided model name and
        mesh type.

        Args:
            model_name (str): The name of the model for which the project structure
                is created. This name is used to define folder paths and file
                content.
            mesh_type (str): The mesh file format to be used in the model (e.g.,
                "stl", "dae").
        """
        # Create the root folder
        if Path(package_path).exists():
            print(f"Found directory: {package_path}")

        # Define subfolders and files

        model_folder = Path(package_path + f"/models/{model_name}")
        world_folder = Path(package_path + f"/worlds")
        meshes_folder = Path(str(model_folder) + f"/meshes")
        if not model_folder.exists():
            model_folder.mkdir(
                parents=True
            )  # Create the folder including any necessary parent directories
            print(f"Folder '{model_folder}' created.")

        if not world_folder.exists():
            world_folder.mkdir(
                parents=True
            )  # Create the folder including any necessary parent directories
            print(f"Folder '{world_folder}' created.")

        if not meshes_folder.exists():
            meshes_folder.mkdir(
                parents=True
            )  # Create the folder including any necessary parent directories
            print(f"Folder '{meshes_folder}' created.")

        config_content = f"""<?xml version="1.0"?>
<model>
<name>{model_name}</name>
<version>1.0</version>
<sdf version="1.7">model.sdf</sdf>

<author>
    <name>Peppermint Robotics</name>
</author>

<description>
Model of {model_name.replace('_', ' ')}.
</description>
</model>
"""

        model_content = f"""<?xml version="1.0"?>
<sdf version="1.7">
    <model name="{model_name}">
        <static>true</static>
        <link name="base">
            <collision name="collision_wall">
                <geometry>
                    <mesh>
                        <uri>meshes/{model_name}_wall.{mesh_type}</uri>
                    </mesh>
                </geometry>
            </collision>
            <collision name="collision_line">
                <geometry>
                    <mesh>
                        <uri>meshes/{model_name}_line.{mesh_type}</uri>
                    </mesh>
                </geometry>
            </collision>
            <visual name="visual_wall">
                <geometry>
                    <mesh>
                        <uri>meshes/{model_name}_wall.{mesh_type}</uri>
                    </mesh>
                </geometry>
            </visual>
            <visual name="visual_line">
                <geometry>
                    <mesh>
                        <uri>meshes/{model_name}_line.{mesh_type}</uri>
                    </mesh>
                </geometry>
            </visual>
        </link>
        <link name="ground_link">
        <collision name="collision1">
        <geometry>
            <plane>
            <normal>0 0 1</normal>
            </plane>
        </geometry>
        </collision>
        <visual name="visual1">
        <geometry>
            <plane>
            <normal>0 0 1</normal>
            <size>300 300</size>
            </plane>
        </geometry>
        <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.9 0 1</diffuse>
            <specular>0 0 0 0</specular>
            <emissive>0.7 0.7 0.7 0.7</emissive>
        </material>
        </visual>
    </link>
    </model>
</sdf>
"""

        world_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
    <world name="{model_name}">
    <!-- Physics -->
        <plugin
        filename="libignition-gazebo-physics-system.so"
        name="ignition::gazebo::systems::Physics">
        </plugin>
        
    <!-- Forwards simulation state to the GUI -->
        <plugin
        filename="libignition-gazebo-scene-broadcaster-system.so"
        name="ignition::gazebo::systems::SceneBroadcaster">
        </plugin>
    
    <!-- Processes user commands, like translate and create -->
        <plugin
        filename="libignition-gazebo-user-commands-system.so"
        name="ignition::gazebo::systems::UserCommands">
        </plugin>
        
    <!-- Generates rendering sensor data -->
        <plugin
        filename="libignition-gazebo-sensors-system.so"
        name="ignition::gazebo::systems::Sensors">
        <render_engine>ogre2</render_engine>
        </plugin>
        
    <scene>
        <ambient>1.0 1.0 1.0 1.0</ambient>
        <background>0.8 0.8 0.8 1.0</background>
        <grid>false</grid>
        <origin_visual>false</origin_visual>
        </scene>
        
        <include>
        <pose>0 0 0 0 0 0</pose>
        <uri>model://{model_name}</uri>
    </include>

    </world>
</sdf>
"""
        # Write placeholder content to the files
        with open(str(model_folder) + f"/model.config", "w") as config_file:
            config_file.write(config_content)

        with open(str(model_folder) + f"/model.sdf", "w") as model_sdf_file:
            model_sdf_file.write(model_content)

        with open(str(world_folder) + f"/{model_name}.sdf", "w") as world_sdf_file:
            world_sdf_file.write(world_content)

        print(f"Project structure created successfully in '{model_name}'.")


def main():
    rclpy.init()
    converter = MapConverter()
    rclpy.logging.get_logger("map2world").info("map2world running")
    rclpy.spin_once(converter)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
