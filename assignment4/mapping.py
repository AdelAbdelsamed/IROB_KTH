#!/usr/bin/env python3

"""
    # {Adel Khaled Mostafa Abdelsamed}
    # {200003221-2412}
    # {akmab@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()

        # Print basic info about the map:
        # print(f' The map height is : {grid_map.get_height()}') Height of all maps is: 300!
        # print(f' The map width is : {grid_map.get_width()}')   Width of all maps is: 300!
        # print(f' The map resolution is : {resolution}') Res of all maps is 0.025
        # print(f' The map origin is : {origin.position}')        


        """
        Fill in your solution here
        """

        # Store all occupied space grids
        occupied_space_grids_list = []
        # Min and max values for update
        min_index_x = 300  # Pick grid boundaries
        min_index_y = 300
        max_index_x = 0  # Pick grid boundaries
        max_index_y = 0



        for c, l_range in enumerate(scan.ranges):

            # Step E.1 and E.2: Convert the laser scan ranges and bearings to coordinates in the laser frame
            #               and Convert the coordinates to the map frame

            # Check to see if scan ranges should be discarded
            if l_range <= scan.range_min or l_range >= scan.range_max:
                continue

            # Obtain laser scan range in x-direction in map frame
            end_scan_map_pos_x = pose.pose.position.x + \
                l_range*cos((scan.angle_min + c*scan.angle_increment) +
                          robot_yaw) - origin.position.x
            # Obtain laser scan range in y-direction in map frame
            end_scan_map_pos_y = pose.pose.position.y + \
                l_range*sin((scan.angle_min + c*scan.angle_increment) +
                          robot_yaw) - origin.position.y

            # Step E.3: Convert the coordinates to map indices

            # Check if position is inside the grid map and obtain the indices if yes
            if self.is_in_bounds(grid_map, end_scan_map_pos_x, end_scan_map_pos_y):
                end_grid_index_x = int(end_scan_map_pos_x / resolution
                                   )
                end_grid_index_y = int(end_scan_map_pos_y / resolution
                                   )

                # Minimum and maximum indices in both x- and y-direction must be occupied grids
                # Keep track of the min and max indices of the updated grids
                if end_grid_index_x < min_index_x: min_index_x = end_grid_index_x
                if end_grid_index_y < min_index_y: min_index_y = end_grid_index_y 
                if end_grid_index_x > max_index_x: max_index_x = end_grid_index_x
                if end_grid_index_y > max_index_y: max_index_y = end_grid_index_y
                
                # Step C.1: Obtain the free cells between robot position and scan endpoints
                
                #  Obtain the start position of the robot for raytracing
                robot_map_pos_x = pose.pose.position.x - origin.position.x
                robot_map_pos_y = pose.pose.position.y - origin.position.y
                # Obtain the start and end grid indices
                start = (int(robot_map_pos_x / resolution), int(robot_map_pos_y / resolution))
                end = (end_grid_index_x, end_grid_index_y)
                # Obtain the traversed grids' indices
                traversed_space_grids = self.raytrace(start, end)
                
                for traversed_grid in traversed_space_grids:
                    # Mark free cells as free in map
                    self.add_to_map(grid_map, traversed_grid[0],
                                    traversed_grid[1], self.free_space)

                # Step E.4: Add the occupied grids for later addition to map (following the hint: order matters!)
                occupied_space_grids_list.append((end_grid_index_x, end_grid_index_y))

        # Step E.4: Mark occupied cells as occupied in map
        for occupied_grid in occupied_space_grids_list:
            self.add_to_map(grid_map, occupied_grid[0],
                                    occupied_grid[1], self.occupied_space)


        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = min_index_x
        # The minimum y index in 'grid_map' that has been updated
        update.y = min_index_y
        # Maximum x index - minimum x index + 1
        update.width = max_index_x - min_index_x + 1
        # Maximum y index - minimum y index + 1
        update.height = max_index_y - min_index_y + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []

        # Add the updated grids in row-major order
        for col in range(update.y, update.y + update.height):
            for row in range(update.x, update.x + update.width):
                update.data.append(grid_map[row, col])

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """
        # print(f'The radius of the robot is {self.radius}') Robot radius is 5! (cells)


        # Define map width and heights
        map_width = 300
        map_height = 300

        for i_x in range(map_width):
            for j_y in range(map_height):

                # 1. First need to check if the corresponding grid is occupied; if not continue
                if grid_map[i_x, j_y] != self.occupied_space:
                    continue

                # 2. Loop over the box with ranges [-self.radius, self.radius] in both directions 
                for r_i_x in range(-self.radius, self.radius + 1):
                    for r_j_y in range(-self.radius, self.radius + 1):

                        # 2.1 Obtain the new grid_indices:
                        new_i_x = i_x + r_i_x
                        new_j_y = j_y + r_j_y

                        # 2.2 Ensure indices lie within the map 
                        if new_i_x < 0 or new_i_x > map_width: continue
                        if new_j_y < 0 or new_j_y > map_height: continue 

                        # 2.3 Check distance to ensure inflated area is a circle not a box
                        if (new_i_x - i_x)**2 + (new_j_y - j_y)**2 > self.radius**2: continue
                        
                        # 2.4 If the grid cell is not occupied space-> mark as c-space 
                        if grid_map[new_i_x, new_j_y] != self.occupied_space:
                            self.add_to_map(grid_map, new_i_x,
                                    new_j_y, self.c_space)
        
        # Return the inflated map
        return grid_map
