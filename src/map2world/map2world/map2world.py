import cv2
import numpy as np
import trimesh
from matplotlib.tri import Triangulation
from pathlib import Path
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid

# Make sure to have pycollada pip package installed for the DAE build to work.


class MapConverter(Node):
    def __init__(self):
        super().__init__("map2world")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("mesh_type", "dae")
        self.declare_parameter("occupied_threshold", 1)
        self.declare_parameter("box_height", 2.0)
        self.declare_parameter("gauss_blur", 7)
        # by increasing blur thresh by greater factor you will make the blurred noise stronger, and for very large value the map will become blank
        self.declare_parameter("edge_strength", 15)
        self.declare_parameter("median_blur", 9)
        # used to decide whether you want to apply slight blur before creating 3d map.
        # blur helps to smudge the edges so they apper to continuous to the contour
        # detection and the gaps in the wall due to sensor error is removed.
        self.declare_parameter("blur_factor", 3)

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
        self.gauss_blur = (
            self.get_parameter("gauss_blur").get_parameter_value().integer_value
        )
        self.edge_strength = (
            self.get_parameter("edge_strength").get_parameter_value().integer_value
        )
        self.median_blur = (
            self.get_parameter("median_blur").get_parameter_value().integer_value
        )
        self.blur_factor = (
            self.get_parameter("blur_factor").get_parameter_value().integer_value
        )

        map_sub_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.test_map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, qos_profile=map_sub_qos
        )
        self.export_dir = str(os.getcwd())
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

        Workflow:
        1. Prompts the user for a model name.
        2. Processes the incoming occupancy grid map to reshape the data.
        3. Converts unknown (-1) map values to unoccupied (0).
        4. Identifies occupied regions using contours.
        5. Generates a 3D mesh model by extruding the contours.
        6. Exports the resulting mesh as STL or DAE based on the user's choice.

        Args:
            map_msg (nav_msgs.msg.OccupancyGrid): ROS OccupancyGrid message containing
            the 2D map data.
        """
        self.get_logger().info("Lets trun 2D map into 3D maps!.")
        model_name = input("\nEnter the model name: ")
        self.create_project_structure(model_name=model_name, mesh_type=self.mesh_type)
        self.get_logger().info("Received map. Processing.")
        map_dims = (map_msg.info.height, map_msg.info.width)
        map_array = np.array(map_msg.data).reshape(map_dims)

        # set all -1 (unknown) values to 0 (unoccupied)
        map_array[map_array < 0] = 0
        contours = self.get_occupied_regions(map_array, model_name)
        meshes = self.contour_to_mesh(contours, map_msg.info)

        mesh = trimesh.util.concatenate(meshes)

        # Export as STL or DAE
        mesh_type = self.mesh_type
        export_dir = self.export_dir
        if mesh_type == "stl":
            with open(
                export_dir + f"/{model_name}/{model_name}/meshes/{model_name}.stl", "wb"
            ) as f:
                mesh.export(f, "stl")
            self.get_logger().info("Exported STL. Shutting down this node now.")
        elif mesh_type == "dae":
            with open(
                export_dir + f"/{model_name}/{model_name}/meshes/{model_name}.dae", "wb"
            ) as f:
                f.write(trimesh.exchange.dae.export_collada(mesh))
            self.get_logger().info("Exported DAE. Shutting down this node now.")

    def get_occupied_regions(self, map_array, model_name):
        """
            Identifies occupied regions in a 2D map using contour detection.

        This function processes the input 2D occupancy grid map by applying several
        image processing techniques, such as thresholding, Gaussian blur, and contour
        detection, to identify regions that are considered occupied. The detected
        contours represent the boundaries of the occupied areas in the map.

        Processing Steps:
            1. Converts the occupancy grid map to an 8-bit unsigned integer format.
            2. Applies a binary threshold to the map to separate occupied and unoccupied regions.
            3. Blurs the thresholded image using a Gaussian filter and applies an inverse
               binary threshold to create a mask.
            4. Applies median blur to the mask and performs a bitwise AND operation with
               the original thresholded image to isolate edges.
            5. Finds contours using OpenCV's `RETR_CCOMP` method, which classifies external
               and internal contours separately.
            6. Saves the contour image for debugging and returns the external contours
               representing occupied regions.

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
        _, thresh_img = cv2.threshold(map_array, self.threshold, 255, cv2.THRESH_BINARY)
        blurred_img = cv2.GaussianBlur(
            thresh_img, (self.gauss_blur, self.gauss_blur), self.gauss_blur
        )
        thresh_blur, thresh_blur_img = cv2.threshold(
            blurred_img, self.edge_strength, 255, cv2.THRESH_BINARY_INV
        )
        medianblur = cv2.medianBlur(thresh_blur_img, self.median_blur)
        final_img = cv2.bitwise_and(thresh_img, medianblur)
        a = cv2.subtract(thresh_img, final_img)
        a = cv2.blur(a, (self.blur_factor, self.blur_factor))

        # Using cv2.RETR_CCOMP classifies external contours at top level of
        # hierarchy and interior contours at second level.
        # If the whole space is enclosed by walls RETR_EXTERNAL will exclude
        # all interior obstacles e.g. furniture.
        # https://docs.opencv.org/trunk/d9/d8b/tutorial_py_contours_hierarchy.html
        contours, hierarchy = cv2.findContours(a, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        contour_img = np.zeros_like(map_array)  # Create a blank image
        # Draw the contours on the blank image
        cv2.drawContours(contour_img, contours, -1, (255, 255, 255), thickness=1)
        # Save the image with contours for debugging
        cv2.imwrite(
            str(os.getcwd()) + f"/{model_name}/contour_image_final.png", contour_img
        )

        hierarchy = hierarchy[0]
        corner_idxs = [i for i in range(len(contours)) if hierarchy[i][3] == -1]

        return [contours[i] for i in corner_idxs]

    def contour_to_mesh(self, contour, metadata):
        """
        Converts 2D contours into a 3D mesh by extruding the contour lines.

        This function takes in a set of 2D contour points and generates a 3D mesh by
        extruding the contour lines vertically. The height of the extrusion is
        determined by a predefined value, and the mesh is constructed using the
        `trimesh` library.

        Processing Steps:
            1. Iterates through each contour and converts the 2D points into real-world
            coordinates using `coords_to_loc`.
            2. Creates line segments from consecutive points in the contour.
            3. Extrudes the 2D line segments into a 3D shape using `trimesh.path.segments.extrude`.
            4. Combines the individual meshes into a single mesh and removes duplicate faces.
            5. Optionally displays the mesh for visualization.

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
        height = np.array([0, 0, self.height])
        s3 = 3**0.5 / 3.0
        meshes = []
        for point in contour:
            new_point_array = []
            for points in point:
                x, y = points[0]
                print(x, y)
                new_point = self.coords_to_loc((x, y), metadata)
                new_point_array.append(new_point)
            # To create a segment we need to pass 2 points as a pair.
            segment = np.array(
                [
                    [new_point_array[i], new_point_array[i + 1]]
                    for i in range(len(new_point_array) - 1)
                ]
            )
            if len(segment) > 0:
                height = self.height
                vertices, faces = trimesh.path.segments.extrude(
                    segments=segment,
                    height=height,
                    double_sided=False,  # Set to True if you want double-sided extrusion
                )
                mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
                meshes.append(mesh)
        mesh = trimesh.util.concatenate(meshes)
        mesh.show()
        mesh.remove_duplicate_faces()

        return mesh

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
        return [loc_x, loc_y]

    def create_project_structure(self, model_name, mesh_type):
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
        if not os.path.exists(model_name):
            os.makedirs(model_name)

        # Define subfolders and files
        model_folder = os.path.join(model_name, f"{model_name}")
        world_folder = os.path.join(model_name, "world")
        meshes_folder = os.path.join(model_folder, "meshes")

        # Create model and world folders
        os.makedirs(model_folder, exist_ok=True)
        os.makedirs(world_folder, exist_ok=True)

        # Create the meshes folder inside the model folder
        os.makedirs(meshes_folder, exist_ok=True)

        # Create the model.config and model.sdf files inside the model folder
        model_config_path = os.path.join(model_folder, "model.config")
        model_sdf_path = os.path.join(model_folder, "model.sdf")
        world_sdf_path = os.path.join(world_folder, f"{model_name}.sdf")

        config_content = f"""<?xml version="1.0"?>
<model>
<name>{model_name}</name>
<version>1.0</version>
<sdf version="1.7">model.sdf</sdf>

<author>
    <name>Peppermint Robotics</name>
    <email>Robotics Team</email>
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
            <collision name="collision">
                <geometry>
                    <mesh>
                        <uri>meshes/{model_name}.{mesh_type}</uri>
                    </mesh>
                </geometry>
            </collision>
            <visual name="visual">
                <geometry>
                    <mesh>
                        <uri>meshes/{model_name}.{mesh_type}</uri>
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
        with open(model_config_path, "w") as config_file:
            config_file.write(config_content)

        with open(model_sdf_path, "w") as model_sdf_file:
            model_sdf_file.write(model_content)

        with open(world_sdf_path, "w") as world_sdf_file:
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
