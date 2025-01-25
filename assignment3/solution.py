#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Adel Khaled Mostafa Abdelsamed}
# {20000321-2412}
# {akmab@kth.se}

from dubins import *
import numpy as np


# Any big number for penalizing collision
INF = 10000


class vertex:

    # Constructor
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

        self.parent = None
        self.control_inputs = (0, 0)


class RTTDubinsCar():

    def __init__(self, car):

        self.car = car

        # Probability to bias the distribution of new nodes
        self.bias_probab = 0.05

        # Maximum simulation steps to compute new path to new node
        self.max_forward_prop_step = 150
        # Tolerance radius for target
        self.target_tol = 1.4

        # Start vertex = Initial position of the car
        self.start_vertex = vertex(self.car.x0, self.car.y0, 0)
        # Target vertex
        self.target_vertex = vertex(
            self.car.xt, self.car.yt, 0)

    def plan_path(self):
        # Implementation follows the pseudocode from lecture 7 slide 40 and Planning book by Lavalle p.833

        # Add the start vertex
        vertices_list = [self.start_vertex]

        i = 1
        while True:
            # 1. Generate random vertex
            rand_vertex = self.generate_random_vertex()

            # 2. Find nearest vertex in vertices_list
            nearest_vertex = self.find_nearest_vertex(
                rand_vertex, vertices_list)

            # 3. Find control input to steer car from nearest_node to rand_vertex
            control_input = self.find_steering_angle(
                nearest_vertex, rand_vertex)

            # 4. Apply control inputs u for max_forward_prop_step and end up in new_vertices
            new_vertex, new_control_inputs, finished = self.forward_propagate_to_new_vertex(
                nearest_vertex, control_input)

            # 5. Add new_node to vertices_list and store the control inputs
            if new_vertex is not None:
                new_vertex.parent = nearest_vertex
                new_vertex.control_inputs = new_control_inputs
                vertices_list.append(new_vertex)

            # 6. Check if target is reached
            if finished:
                # Construct the control input sequence
                control_sequence = self.generate_control_path(
                    new_vertex)

                # Construct the time vector
                times = [0.0]
                for u in control_sequence:
                    times.append(times[-1] + 0.01)

                # # Printing the target and actual end positions
                # print(f"Car end Position in x-direction: {new_vertex.x}")
                # print(f"Car end Position in y-direction: {new_vertex.y}")
                # print(
                #     f"Car target Position in x-direction: {self.target_vertex.x}")
                # print(
                #     f"Car target Position in y-direction: {self.target_vertex.y}")

                return control_sequence, times

            # # For keeping track of progress
            # if (i % 1000) == 0:
            #     # Obtain the distance of the vertex reaching the target
            #     dist_best_vertex = self.best_vertex_to_target(vertices_list)
            #     print(
            #         f"Iter {i}: Planning... || Closest vertex to target: {dist_best_vertex}")

            i = i+1

    def generate_random_vertex(self):
        """
        Returns a random vertex with proabability p% and with probability (100-p)% returns the target vertex
        """
        # Idea about biasing the distribution stems from Lecture 07 Slide 39
        r = np.random.uniform()    # Get random uniform number between [0, 1)

        if r > self.bias_probab:
            # Create random vertex within the bounds of the world
            random_vertex = vertex(np.random.uniform(self.car.xlb, self.car.xub),
                                   np.random.uniform(
                                       self.car.ylb, self.car.yub),
                                   0)
        else:
            random_vertex = self.target_vertex
        return random_vertex

    def find_nearest_vertex(self, rand_vertex, vertices_list):
        """
        Returns the nearest vertex in vertices_list to rand_vertex
        """
        # Distance function \rho used here is simple: euclidean distance between (x,y) positions of the vertices (might change later)
        # Utilize numpy vector instead of lists for efficiency
        vertices_array = np.array([(v.x, v.y) for v in vertices_list])
        rand_vertex_array = np.array([rand_vertex.x, rand_vertex.y])

        # Compute the squared Euclidean distances for each vertex as a row vector
        dist_squared = np.sum(
            (vertices_array - rand_vertex_array) ** 2, axis=1)

        # Return the nearest vertex using np.argmin to obtain the index
        return vertices_list[np.argmin(dist_squared)]

    def find_steering_angle(self, nearest_vertex, rand_vertex):
        """
        Returns the steering angle necessary for the car to go from
        nearest vertex to rand_vertex
        """
        # Implementation follows Lecture 07 Slide 40
        # Avoid np.linalg.norm for computational efficiency!
        # Helper variables for computation of steering angle
        dx = rand_vertex.x - nearest_vertex.x
        dy = rand_vertex.y - nearest_vertex.y
        norm_rg = np.sqrt(dx ** 2 + dy ** 2)

        v_x = np.cos(nearest_vertex.theta)
        v_y = np.sin(nearest_vertex.theta)

        cross_product = v_x * dy - v_y * dx
        dot_product = v_x * dx + v_y * dy
        # Compute sin_phi frpm cross product equation as
        sin_phi = cross_product / norm_rg
        # Compute cos_phi from dot product equation as
        cos_phi = dot_product / norm_rg
        # Compute the steering angle as:
        phi = np.arctan2(sin_phi, cos_phi)
        # Return clipped steering angle
        return np.clip(phi, -np.pi / 4, np.pi / 4)

    def collision_avoidance(self, x, y, tol_margin=0.05):
        """
        Returns True if it is safe to be in location x, y and False if collision is about to occur
        """
        # This function follows the same idea as the RRT implementation of the GitHub repo AtsushiSakai/PythonRobotics/Planning/RRT
        # See if [x,y] is within a radius r close to any obstacle
        # Optimize efficiency with numpy vector operations

        # 1. Transform the obstacles into an array [n_obs, 3]
        obs_array = np.array(self.car.obs)

        # 2. Compute distance to (x,y) as a row vector
        dists_to_obs = np.sqrt(
            (obs_array[:, 0] - x) ** 2 + (obs_array[:, 1] - y) ** 2)

        # 3. Check for any collision by checking if any row returns True
        collision = np.any(dists_to_obs <= obs_array[:, 2] + tol_margin)
        # 4. Return true if no collisions
        return not collision

    def dist_to_target(self, x, y):
        """
        Returns the distance to the target vertex
        """
        # Use np.hypot for vectorized operations for computational speed
        return np.hypot(self.target_vertex.x - x, self.target_vertex.y - y)

    def out_of_world_avoidance(self, x, y, tol_margin=0.05):
        """
        Returns True if it is inside the bounds (with some margin, chosen empirically) 
        defined and False if it is out of the defined world
        """
        if (self.car.xlb) < x and (self.car.ylb + tol_margin) < y and (self.car.xub) > x and (self.car.yub - tol_margin) > y:
            return True
        else:
            return False

    def best_vertex_to_target(self, vertices_list):
        """
        Returns the distance of the closest vertex to the target (for tarcking progress)
        """
        # Penalize vertices if colliding
        dist_vertices_to_target = [self.dist_to_target(vertex.x, vertex.y) if (self.out_of_world_avoidance(
            vertex.x, vertex.y) and self.collision_avoidance(vertex.x, vertex.y)) else INF for vertex in vertices_list]

        min_dist_vertices_to_target = min(dist_vertices_to_target)
        return min_dist_vertices_to_target

    def generate_control_path(self, target_vertex):
        """
        Returns the entire sequence of controls from start to target
        """
        # This function follows the same idea as the RRT implementation of the GitHub repo AtsushiSakai/PythonRobotics/Planning/RRT
        # In particular it follows the function get_final_course

        control_sequence = []
        path = [target_vertex]

        # Iterate until start node is reached; start node has no parent node
        while path[-1].parent is not None:
            # Set the current vertex
            curr_vertex = path[-1]

            # Obtain the control inputs and steps
            phi, steps = curr_vertex.control_inputs

            # Add controls
            control_sequence.extend([phi] * steps)

            # Add parent to path
            path.append(curr_vertex.parent)

        return list(reversed(control_sequence))

    def forward_propagate_to_new_vertex(self, nearest_vertex, control_input):
        """
        Returns the stored control inputs and the new vertex after
        checking collision and bounds
        """
        # Obtain the control inputs
        phi = control_input
        # Obtain position of the nearest vertex
        x, y, theta = nearest_vertex.x, nearest_vertex.y, nearest_vertex.theta

        for n in range(0, self.max_forward_prop_step):
            x_new, y_new, theta_new = step(
                self.car, x, y, theta, phi, dt=0.01)
            # Update state for next iteration
            x, y, theta = x_new, y_new, theta_new

            # Check if point collides or is out of bounds or goal reached; then stop simulation
            target_reached = self.dist_to_target(
                x_new, y_new) < self.target_tol
            move_is_safe = (self.out_of_world_avoidance(
                x_new, y_new) and self.collision_avoidance(x_new, y_new))
            if not move_is_safe or target_reached:
                break

        if target_reached and move_is_safe:
            # Create new vertex with goal vertex
            print("Target reached! Generating the control sequence...")
            new_vertex = vertex(x_new, y_new, theta_new)
            required_control = (phi, n + 1)
            return new_vertex, required_control, True

        # In case of an obstacle or an unsafe zone, do not move at all; instead dismiss vertex
        if n != (self.max_forward_prop_step - 1):
            return None, None, False

        # Create new vertex
        if move_is_safe:
            new_vertex = vertex(x_new, y_new, theta_new)
            required_control = (phi, n + 1)
            return new_vertex, required_control, False
        else:
            return None, None, False


def solution(car):
    '''
    Your code below
    '''
    # Create RRT DubinsCar object
    rrt_dubin_planner = RTTDubinsCar(car)
    # Call path planning algorithm (RRT)
    controls, times = rrt_dubin_planner.plan_path()

    '''
  Your code above
  '''

    return controls, times
