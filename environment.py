import pybullet as p
import random
from urdfenvs.robots.generic_urdf.generic_diff_drive_robot import GenericDiffDriveRobot
import math
from maze import generate_maze

class Environment:
    def __init__(self):
        self.bound_boxes = list()
        self.add_colored_plane(position=[0, 0, 0], orientation=[0, 0, 0, 1],
                plane_size=[15, 15, 0.01], plane_color=[0, 0, 0.2, 1.0])
        self.manipulatorObstacle()
        self.create_maze()
        # self.create_triangle()

    @staticmethod
    def distance(pos1, pos2):
        return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5

    def add_colored_plane(self, position, orientation, plane_size=[30, 30, 0.1], plane_color=[0, 0, 0.2, 1.0]):
        """
        Add a colored plane to the environment as a floor.
        :param position: Position of the plane (x, y, z).
        :param orientation: Orientation of the plane (quaternion).
        :param plane_size: Size of the plane (x, y, z).
        :param plane_color: Color of the plane in RGBA format.
        """
        # Create the visual shape for the colored plane
        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=plane_size, rgbaColor=plane_color)

        # Create the collision shape for the plane
        collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=plane_size)

        # Create the plane using the visual and collision shapes
        plane_body_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape_id,
                                          baseVisualShapeIndex=visual_shape_id, basePosition=position,
                                          baseOrientation=orientation)

    def remove_bounding_boxes(self):
        for box_id in self.bound_boxes:
            p.removeBody(box_id)

    def create_object_with_invisible_bounds(self, position, orientation, visual_half_extents, collision_extension=0.5,
                                            show_bounds=False):
        """
        Create an object and optionally an associated invisible bounding box as separate bodies.
        :param position: Position of the object (x, y, z).
        :param orientation: Orientation of the object (quaternion).
        :param visual_half_extents: Half extents of the visual shape (x, y, z).
        :param collision_extension: Additional size added to each dimension for the collision shape.
        :param show_bounds: If True, show the bounding box with opacity 0.3.
        :return: Tuple containing the Body ID of the created object and the bounding box (if created).
        """
        # Create the visual shape for the object
        visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=visual_half_extents)

        # Create the collision shape for the object
        collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=visual_half_extents)

        # Create the object with the visual and collision shape
        object_body_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape_id,
                                           baseVisualShapeIndex=visual_shape_id, basePosition=position,
                                           baseOrientation=orientation)

        bounding_box_body_id = -1
        if show_bounds:
            # Create an extended collision shape for the bounding box
            collision_half_extents = [x + collision_extension for x in visual_half_extents]
            bounds_collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=collision_half_extents)

            # Create a semi-transparent visual shape for the bounding box
            bounds_visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=collision_half_extents,
                                                         rgbaColor=[1, 0, 0, 0.3])

            # Create the bounding box as a separate body with no mass and only a visual shape
            bounding_box_body_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=bounds_collision_shape_id ,
                                                     # No collision for bounding box
                                                     baseVisualShapeIndex=bounds_visual_shape_id, basePosition=position,
                                                     baseOrientation=orientation)
        self.bound_boxes.append(bounding_box_body_id)
        return object_body_id, bounding_box_body_id

    def create_maze(self, start_position=(2.0, 0), goal_position=(-14.0, -14.0), threshold_distance=4.0):
        # Wall dimensions and positions for outer walls
        wall_half_extents = [15, 0.1, 0.5]
        wall_positions = [[0, 15, 0.5], [0, -15, 0.5], [15, 0, 0.5], [-15, 0, 0.5]]
        wall_orientations = [[0, 0, 0], [0, 0, 0], [0, 0, 1.57], [0, 0, 1.57]]

        # Create outer walls
        for pos, ori in zip(wall_positions, wall_orientations):
            ori_quat = p.getQuaternionFromEuler(ori)
            self.create_object_with_invisible_bounds(pos, ori_quat, wall_half_extents, show_bounds=True)

        # Maze walls dimensions and grid size
        wall_half_extents = [0.5, 0.5, 0.3]
        grid_size_x = int(30 / (wall_half_extents[0] + 1))
        grid_size_y = int(30 / (wall_half_extents[1] + 1))
        maze_grid = generate_maze( start_position, goal_position, grid_size_x=grid_size_x, grid_size_y=grid_size_y)

        # Function to calculate distance between two points
        def distance(pos1, pos2):
            return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5

        # Create maze walls
        walls = []
        for y in range(grid_size_y):
            for x in range(grid_size_x):
                wall_position = [x * (1 + wall_half_extents[0]) - 14.5, y * (1 + wall_half_extents[1]) - 14.5,
                                 wall_half_extents[2]]

                # Check if the position is within the threshold distance from start or goal
                if maze_grid[y][x] == 1 and distance(wall_position[:2],
                                                     start_position) >= threshold_distance and distance(
                        wall_position[:2], goal_position) >= threshold_distance:
                    wall_orientation = [0, 0, 0]  # Assuming no rotation
                    ori_quat = p.getQuaternionFromEuler(wall_orientation)

                    wall_id = self.create_object_with_invisible_bounds(
                        wall_position, ori_quat, wall_half_extents,
                        collision_extension=0.4, show_bounds=True
                    )
                    walls.append(wall_id)

        return walls

    def create_maze_old(self):
        # Wall dimensions and positions
        wall_half_extents = [15, 0.1, 0.5]  # Modify these values as needed
        wall_positions = [
            [-1, 15, 0.5], [0, -15, 0.5], [15, 0, 0.5], [-15, 0, 0.5]
        ]
        wall_orientations = [
            [0, 0, 0], [0, 0, 0], [0, 0, 1.57], [0, 0, 1.57]
        ]

        # Create walls
        for pos, ori in zip(wall_positions, wall_orientations):
            ori_quat = p.getQuaternionFromEuler(ori)
            self.create_object_with_invisible_bounds(pos, ori_quat, wall_half_extents, show_bounds=True)

        grid_size = 29
        num_taskspace_objects = 50
        taskspace_half_extents = [0.7, 0.1, 0.5]
        threshold_distance = 0.40
        threshold_distance_goal_start = 4.0 # Distance threshold within which no other object can be placed
        start_position = (2.0, 0)  # Replace with actual start coordinates
        goal_position = (-14.0, -14.0)

        # Generate all possible positions in the grid
        all_positions = [(x, y) for x in range(grid_size) for y in range(grid_size)]

        # Function to calculate distance between two points
        def distance(pos1, pos2):
            return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5

        # Remove positions too close to the start or goal
        all_positions = [pos for pos in all_positions if
                         distance(pos, start_position) >= threshold_distance_goal_start and distance(pos,
                                                                                                     goal_position) >= threshold_distance_goal_start]
        print('after filtering:')

        selected_positions = []

        while len(selected_positions) < num_taskspace_objects and all_positions:
            pos = random.choice(all_positions)
            selected_positions.append(pos)

            # Remove positions within the threshold distance of the selected position
            all_positions = [p for p in all_positions if
                             math.sqrt((p[0] - pos[0]) ** 2 + (p[1] - pos[1]) ** 2) > threshold_distance]

        for pos in selected_positions:
            world_x = pos[0] - grid_size / 2
            world_y = pos[1] - grid_size / 2
            world_position = [world_x, world_y, 0.5]  # Adjust z-coordinate as needed

            # Randomize yaw orientation
            random_yaw = random.uniform(0, 2 * math.pi)
            ori_quat = p.getQuaternionFromEuler([0, 0, random_yaw])

            self.create_object_with_invisible_bounds(world_position, ori_quat, taskspace_half_extents,
                                                     show_bounds=True)

    def create_triangle(self):
        t_visual_half_extents = [1, 0.05, 0.5]  # Adjust these values as needed
        show_bounds = True
        # Triangle Part 1
        t_one_position = [0, 0.8, 0.5]
        t_one_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.create_object_with_invisible_bounds(t_one_position, t_one_orientation, t_visual_half_extents,show_bounds=show_bounds)

        # Triangle Part 2
        t_two_position = [0.5, -0.13, 0.5]
        t_two_orientation = p.getQuaternionFromEuler([0, 0, 1.047])
        self.create_object_with_invisible_bounds(t_two_position, t_two_orientation, t_visual_half_extents,show_bounds=show_bounds)

        # Triangle Part 3
        t_three_position = [-0.5, -0.13, 0.5]
        t_three_orientation = p.getQuaternionFromEuler([0, 0, -1.047])
        self.create_object_with_invisible_bounds(t_three_position, t_three_orientation, t_visual_half_extents,show_bounds=show_bounds)

    def apply_robot_velocities(self, robot_id, wheel_velocities):
        """Apply velocities to the robot's wheels."""
        # Assuming indices 3 and 4 are for the right and left wheels respectively
        p.setJointMotorControl2(bodyUniqueId=robot_id,
                                jointIndex=3,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=wheel_velocities[0])
        p.setJointMotorControl2(bodyUniqueId=robot_id,
                                jointIndex=4,
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=wheel_velocities[1])
        
    def manipulatorObstacle(self):
        #Object 1
        obj_one_half_extents = [0.15, 0.5, 0.2]  
        obj_one_collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=obj_one_half_extents)
        obj_one_visual_id = 1  

        obj_one_position = [0.25, 0, 0.2]
        obj_one_orientation = p.getQuaternionFromEuler([0, 0, 0])  
        obj_one_id = p.createMultiBody(0, obj_one_collision_id, obj_one_visual_id, obj_one_position, obj_one_orientation)
    
        #Object 2
        obj_one_half_extents = [0.15, 0.05, 0.08]  
        obj_one_collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=obj_one_half_extents)
        obj_one_visual_id = 1  

        obj_one_position = [0.25, 0, 0.48]
        obj_one_orientation = p.getQuaternionFromEuler([0, 0, 0])  
        obj_one_id = p.createMultiBody(0, obj_one_collision_id, obj_one_visual_id, obj_one_position, obj_one_orientation)

        #object 3
        obj_one_half_extents = [0.1, 0.45, 0.05]  
        obj_one_collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=obj_one_half_extents)
        obj_one_visual_id = 1  

        obj_one_position = [0.25, 0, 0.655]
        obj_one_orientation = p.getQuaternionFromEuler([0, 1.57, 0])  
        obj_one_id = p.createMultiBody(0, obj_one_collision_id, obj_one_visual_id, obj_one_position, obj_one_orientation)

    def getObjects(self):
        object_id = []
        num_objects = p.getNumBodies()
        # print("Number of objects in the environment:", num_objects)

        for i in range(num_objects):
            body_info = p.getBodyInfo(i)
            
            if body_info[1].decode('utf-8') == "youbot" or body_info[1].decode('utf-8') == "plane":
                continue
            else:
                object_id.append(i)

        return object_id