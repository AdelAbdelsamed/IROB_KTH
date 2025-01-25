#!/usr/bin/env python3
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot, sqrt

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def move(path):
    global control_client, robot_frame_id, pub

    # Call service client with path
    rospy.wait_for_service('get_setpoint')
    service_response = control_client(path)

    setpoint = service_response.setpoint
    new_path = service_response.new_path

    # Transform Setpoint from service client
    transform = tf_buffer.lookup_transform('base_link', setpoint.header.frame_id, rospy.Time())
    transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)

    # Create Twist message from the transformed Setpoint
    msg = Twist()

    msg.angular.z = 4* atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
    # Clamp the angular velocity:
    if msg.angular.z >= max_angular_velocity:
        msg.angular.z = max_angular_velocity

    msg.linear.x = 0.5 * sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y **2)
    # Clamp the linear velocity:
    if msg.linear.x >= max_linear_velocity:
        msg.linear.x = max_linear_velocity
    
    # To ensure the robot does not get stuck in walls
    if msg.angular.z >= (max_angular_velocity - 0.1):
        msg.linear.x = 0

    # Publish Twist
    pub.publish(msg)

    # Call service client again if the returned path is not empty and do stuff again
    service_response = control_client(new_path)
    setpoint = service_response.setpoint
    new_path = service_response.new_path

    while new_path.poses:
        # Transform Setpoint from service client
        transform = tf_buffer.lookup_transform('base_link', setpoint.header.frame_id, rospy.Time())
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)

        # Create Twist message from the transformed Setpoint
        msg = Twist()

        msg.angular.z = 4* atan2(transformed_setpoint.point.y, transformed_setpoint.point.x)
        # Clamp the angular velocity:
        if msg.angular.z >= max_angular_velocity:
            msg.angular.z = max_angular_velocity

        msg.linear.x = 0.5 * sqrt(transformed_setpoint.point.x ** 2 + transformed_setpoint.point.y **2)
        # Clamp the linear velocity:
        if msg.linear.x >= max_linear_velocity:
            msg.linear.x = max_linear_velocity

        # To ensure the robot does not get stuck in walls
        if msg.angular.z >= (max_angular_velocity - 0.1):
            msg.linear.x = 0
        
        # Publish Twist
        pub.publish(msg)

        service_response = control_client(new_path)
        setpoint = service_response.setpoint
        new_path = service_response.new_path

        rate.sleep()

    # Send 0 control Twist to stop robot
    msg = Twist()
    msg.angular.z = 0
    msg.linear.x =  0
    # Publish Twist
    pub.publish(msg)


def get_path():
    global goal_client

    # Loop until RH-NBV outputs 0 gain or an empty path
    while True:
        # Get path from action server
        goal_client.wait_for_server()          # Wait for the server to start up and listen for goals

        goal_client.send_goal(0)               # Send goal to action server; action server expects no goals

        goal_client.wait_for_result()          # Wait for server to finish performing action

        result = goal_client.get_result()      # result of action

        gain = result.gain
        path = result.path
        
        print(path.poses)
        if (gain == 0) or (not path.poses):    # terminating condition: either gain is zero or path is empty!
            break;

        #rospy.loginfo(f"Gain from action server: {gain}")
        #rospy.loginfo(f"Path from action server: {path}")

        # Call move with path from action server
        move(path)


if __name__ == "__main__":
    # Init node
    rospy.init_node('controller_node_py')
    print("controller_node.pz successfully initialized!")

    # Init publisher
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 15)
    rate = rospy.Rate(10)

    # Init simple action client
    goal_client = actionlib.SimpleActionClient('get_next_goal', irob_assignment_1.msg.GetNextGoalAction)
    print("Action Client successfully initialized!")

    # Init service client
    control_client = rospy.ServiceProxy('get_setpoint', irob_assignment_1.srv.GetSetpoint)
    print("Service Client successfully initialized!")

    # Call get path
    get_path()
    print("path-call success!")

    # Spin
    rospy.spin()
