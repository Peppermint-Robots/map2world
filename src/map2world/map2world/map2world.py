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

        # Declare Parameters
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("mesh_type", "dae")
        self.declare_parameter("occupied_threshold", 1)
        self.declare_parameter("box_height", 2.0)
        self.declare_parameter("package_name", "sim_models_ppmt")
        self.declare_parameter("model_name", "new_model")
        self.declare_parameter("img_path", "-1")
        self.declare_parameter("map_mode", "clean")
        self.declare_parameter("red", 255)
        self.declare_parameter("green", 0)
        self.declare_parameter("blue", 0)

        # Get Parameters
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
        self.img_path = (
            self.get_parameter("img_path").get_parameter_value().string_value
        )
        self.map_mode = (
            self.get_parameter("map_mode").get_parameter_value().string_value
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

        self.template_path = (
            str(get_package_share_directory("map2world")) + "/templates/"
        )

    def map_callback(self, map_msg):
        """Processes a 2D map message and converts it into a 3D map by generating
            3D meshes (STL or DAE) for the occupied regions. The method also exports
            the generated 3D meshes to the appropriate file format.

        Args:
            map_msg (MapMessage): The map message containing the 2D occupancy grid
            with the map's metadata (e.g., width, height, resolution).
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

        if self.map_mode == "line":
            if self.img_path == "-1":
                self.img_path = input("Please provide image path: ")
        
        if self.map_mode == "only_line":
            if self.img_path == "-1":
                self.img_path = input("Please provide image path: ")

        contours = self.get_occupied_regions(map_array, self.img_path)
        meshes = self.contour_to_mesh(contours, map_msg.info)

        mesh_wall = trimesh.util.concatenate(meshes[0])
        mesh_line = trimesh.util.concatenate(meshes[1])

        # Export as STL or DAE
        mesh_type = self.mesh_type

        if mesh_type == "stl":
            with open(
                package_path + f"/models/{model_name}/meshes/{model_name}_wall.stl",
                "wb",
            ) as f:
                mesh_wall.export(f, "stl")
            if self.map_mode == "line":
                with open(
                    package_path + f"/models/{model_name}/meshes/{model_name}_line.stl",
                    "wb",
                ) as f:
                    mesh_line.export(f, "stl")

            self.get_logger().info("Exported STL. Shutting down this node now.")

        elif mesh_type == "dae":
            if self.map_mode == "clean":
                with open(
                    package_path + f"/models/{model_name}/meshes/{model_name}_wall.dae",
                    "wb",
                ) as f:
                    f.write(trimesh.exchange.dae.export_collada(mesh_wall))
            if self.map_mode == "line":
                with open(
                    package_path + f"/models/{model_name}/meshes/{model_name}_wall.dae",
                    "wb",
                ) as f:
                    f.write(trimesh.exchange.dae.export_collada(mesh_wall))
                    
                with open(
                    package_path + f"/models/{model_name}/meshes/{model_name}_line.dae",
                    "wb",
                ) as f:
                    f.write(trimesh.exchange.dae.export_collada(mesh_line))
            if self.map_mode == "only_line":
                with open(
                    package_path + f"/models/{model_name}/meshes/{model_name}_line.dae",
                    "wb",
                ) as f:
                    f.write(trimesh.exchange.dae.export_collada(mesh_line))

            self.get_logger().info("Exported DAE. Shutting down this node now.")

    def get_occupied_regions(self, map_array, img_loc):
        """Identifies and returns the occupied regions (walls and/or paths) in the map using contours.
           Depending on the `map_mode` ("clean" or "line"), it processes the map differently and extracts the
           necessary contours for 3D mesh generation.

        Args:
            map_array (numpy.ndarray): A 2D numpy array representing the map's occupancy grid.
            img_loc (str): The file path to an image used when `map_mode` is "line".

        Returns:
            list: Contours list representing the contours of walls in the map.
            and second contours list representing the lines (paths) if `map_mode` is "line",
            otherwise an empty string ("").
        """

        map_array = map_array.astype(np.uint8)

        if self.map_mode == "clean":

            # Using cv2.RETR_CCOMP classifies external contours at top level of
            # hierarchy and interior contours at second level.
            # If the whole space is enclosed by walls RETR_EXTERNAL will exclude
            # all interior obstacles e.g. furniture.
            # https://docs.opencv.org/trunk/d9/d8b/tutorial_py_contours_hierarchy.html

            contours_wall, hierarchy_wall = cv2.findContours(
                map_array, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
            )

            hierarchy_wall = hierarchy_wall[0]
            corner_idxs_wall = [
                i for i in range(len(contours_wall)) if hierarchy_wall[i][3] == -1
            ]

            return [[contours_wall[i] for i in corner_idxs_wall], ""]

        if self.map_mode == "line":

            img = cv2.imread(img_loc)
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            img_gray[(5 < img_gray) & (img_gray < 250)] = (
                255  # used to remove the path to identify walls
            )
            img_gray = cv2.flip(
                img_gray, 0
            )  # flipping the image so that its aligned with map_array
            img_gray = cv2.bitwise_not(img_gray)

            img_walls = cv2.bitwise_and(img_gray, map_array)
            contours_wall, hierarchy_wall = cv2.findContours(
                img_walls, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
            )

            hierarchy_wall = hierarchy_wall[0]
            corner_idxs_wall = [
                i for i in range(len(contours_wall)) if hierarchy_wall[i][3] == -1
            ]

            hsvFrame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            red_lower = np.array([0, 120, 70], np.uint8)
            red_upper = np.array([180, 255, 255], np.uint8)
            img_path = cv2.inRange(hsvFrame, red_lower, red_upper)
            img_path = cv2.flip(
                img_path, 0
            )  # flipping the image so that its aligned with map_array

            contours_line, hierarchy_line = cv2.findContours(
                img_path, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
            )

            hierarchy_line = hierarchy_line[0]
            corner_idxs_line = [
                i for i in range(len(contours_line))
            ]
            return [
                [contours_wall[i] for i in corner_idxs_wall],
                [contours_line[i] for i in corner_idxs_line],
            ]
        if self.map_mode == "only_line":

            img = cv2.imread(img_loc)
            
            hsvFrame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            red_lower = np.array([0, 120, 70], np.uint8)
            red_upper = np.array([180, 255, 255], np.uint8)
            img_path = cv2.inRange(hsvFrame, red_lower, red_upper)
            img_path = cv2.flip(
                img_path, 0
            )  # flipping the image so that its aligned with map_array

            contours_line, hierarchy_line = cv2.findContours(
                img_path, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
            )

            hierarchy_line = hierarchy_line[0]
            corner_idxs_line = [
                i for i in range(len(contours_line))
            ]
            return [
                "",
                [contours_line[i] for i in corner_idxs_line],
            ]

    def contour_to_mesh(self, contour, metadata):
        """Converts 2D contours into 3D meshes by extruding the contour polygons.
           This method processes both wall and line contours (if present) and generates
           3D meshes using the `trimesh` library.

        Args:
            contour (list): A list containing two elements:
            - First element (list): Contours representing walls (required).
            - Second element (list): Contours representing lines or paths (optional, may be an empty string).
            metadata (MapMetaData): Metadata from the map

        Returns:
            list: A list containing two elements:
            - First element (list): A list of `trimesh` meshes representing the walls.
            - Second element (list): A list of `trimesh` meshes representing the lines (paths) if any, otherwise an empty list.
        """

        meshes_line = []
        meshes_wall = []
        if contour[0] != "":
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
                    pixel_polygon = Polygon(
                        [
                            (x, y),
                            (x + pixel_size, y),
                            (x + pixel_size, y + pixel_size),
                            (x, y + pixel_size),
                        ]
                    )

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
                    pixel_polygon = Polygon(
                        [
                            (x, y),
                            (x + pixel_size, y),
                            (x + pixel_size, y + pixel_size),
                            (x, y + pixel_size),
                        ]
                    )

                    mesh_line = trimesh.creation.extrude_polygon(pixel_polygon, height)
                    face_colors = [self.red, self.green, self.blue]
                    mesh_line.visual.face_colors = face_colors
                    meshes_line.append(mesh_line)

        mesh = trimesh.util.concatenate(meshes_wall, meshes_line)
        print(
            "If you want to discard the mesh file press CTRL+C or else close the 3D preview using GUI"
        )
        mesh.show()
        mesh.remove_duplicate_faces()

        return [meshes_wall, meshes_line]

    def coords_to_loc(self, coords, metadata):
        """Converts pixel coordinates in the map to world coordinates using the map's metadata.

        Args:
            coords (tuple): A tuple of (x, y) representing pixel coordinates in the 2D map grid.
            metadata (MapMetaData): Metadata from the map.

        Returns:
            tuple: A tuple of (loc_x, loc_y), the world coordinates corresponding to the input
            pixel coordinates.
        """
        x, y = coords
        loc_x = x * metadata.resolution + metadata.origin.position.x
        loc_y = y * metadata.resolution + metadata.origin.position.y
        # TODO: transform (x*res, y*res, 0.0) by Pose map_metadata.origin
        # instead of assuming origin is at z=0 with no rotation wrt map frame
        return (loc_x, loc_y)

    def write_model_data(self, model_name, mesh_type, model_folder, world_folder):
        """Writes model and world data to SDF and configuration files by copying content
           from template files and replacing placeholders with the specified model name and mesh type.

        Args:
            model_name (str): The name of the model, used for replacing placeholders in the template files.
            mesh_type (str): The type of mesh ("stl" or "dae") used in the model, replaced in the model SDF file.
            model_folder (str): The folder path where the model files (SDF and config) will be saved.
            world_folder (str): The folder path where the world SDF file will be saved.
        """

        # Path to the template .sdf file
        world_path = os.path.expanduser(self.template_path + "model_world.sdf")

        with open(world_path, "r") as world_file:
            world_content = world_file.read()

        world_content = world_content.replace("{model_name}", model_name)

        with open(str(world_folder) + f"/{model_name}.sdf", "w") as world_sdf_file:
            world_sdf_file.write(world_content)

        config_path = os.path.expanduser(self.template_path + "model.config")

        with open(config_path, "r") as config_file:
            config_content = config_file.read()

        config_content = config_content.replace("{model_name}", model_name)

        with open(str(model_folder) + f"/model.config", "w") as config_file:
            config_file.write(config_content)

        if self.map_mode == "clean":
            model_path = os.path.expanduser(self.template_path + "model_clean.sdf")

            with open(model_path, "r") as model_file:
                model_content = model_file.read()

            model_content = model_content.replace("{model_name}", model_name)
            model_content = model_content.replace("{mesh_type}", mesh_type)

            with open(str(model_folder) + f"/model.sdf", "w") as model_sdf_file:
                model_sdf_file.write(model_content)

        if self.map_mode == "line":
            model_path = os.path.expanduser(self.template_path + "model_line.sdf")

            with open(model_path, "r") as model_file:
                model_content = model_file.read()

            model_content = model_content.replace("{model_name}", model_name)
            model_content = model_content.replace("{mesh_type}", mesh_type)

            with open(str(model_folder) + f"/model.sdf", "w") as model_sdf_file:
                model_sdf_file.write(model_content)
        
        if self.map_mode == "only_line":
            model_path = os.path.expanduser(self.template_path + "model_only_line.sdf")

            with open(model_path, "r") as model_file:
                model_content = model_file.read()

            model_content = model_content.replace("{model_name}", model_name)
            model_content = model_content.replace("{mesh_type}", mesh_type)

            with open(str(model_folder) + f"/model.sdf", "w") as model_sdf_file:
                model_sdf_file.write(model_content)

    def create_project_structure(self, model_name, mesh_type, package_path):
        """Creates the directory structure for a new 3D model project, including model, world, and mesh folders,
           and writes necessary model and world data files based on templates.

        Args:
            model_name (str): The name of the model to be used in file paths and for naming the model folder.
            mesh_type (str): The type of mesh ("stl" or "dae") used in the model.
            package_path (str): The base directory where the project structure will be created
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

        self.write_model_data(model_name, mesh_type, model_folder, world_folder)


def main():
    rclpy.init()
    converter = MapConverter()
    rclpy.logging.get_logger("map2world").info("map2world running")
    rclpy.spin_once(converter)
    rclpy.shutdown()


if __name__ == "__main__":
    main()