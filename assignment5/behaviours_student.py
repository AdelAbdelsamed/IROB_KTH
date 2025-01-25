# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.

import numpy as np
import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseArray
import math

# task_finished = False
class counter(pt.behaviour.Behaviour):

    """
    Returns failure for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class wait(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising wait behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(wait, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.RUNNING if self.i <= self.n else pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class movecube(pt.behaviour.Behaviour):

    """
    Pick or place the cube.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, operation):

        rospy.loginfo("Initialising move cube behaviour.")

        # server
        mv_cube_srv = '/' + operation + '_srv'
        mv_cube_srv_nm = rospy.get_param(rospy.get_name() + mv_cube_srv)
        self.move_cube_srv = rospy.ServiceProxy(mv_cube_srv_nm, SetBool)
        rospy.wait_for_service(mv_cube_srv_nm, timeout=30)

        self.operation = operation

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movecube, self).__init__("GET EM!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_cube_req = self.move_cube_srv()
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_cube_req.success:
            rospy.loginfo("Robot finished: " + self.operation + " the cube!")
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_cube_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class detectcube(pt.behaviour.Behaviour):

    """
    Sends a goal to the play motion action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising detect cube behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server")
            exit()
        rospy.loginfo("%s: Connected to play_motion action server")

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'inspect_surroundings' # Try to detect the cube from the surroundings
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(detectcube, self).__init__("Detect cube!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:
            rospy.loginfo('Detecting cube now!')
            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING


class cube_detected_on_table2(pt.behaviour.Behaviour):

    """
    Returns if cube is placed on table 2.
    """

    def __init__(self, mode = 0):

        rospy.loginfo("Initialising placement detection of cube on table 2.")

        #global task_finished

        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        self.detected_cube = False
        self.listened = False

        #self.mission_done = mission_done
        self.mode = mode

        # become a behaviour
        super(cube_detected_on_table2, self).__init__("Is_job_done")

    def update(self):

        # If listened return result
        if self.listened:
            if self.detected_cube:
                return pt.common.Status.SUCCESS
            elif (not self.detected_cube) and self.mode == 0:
                return pt.common.Status.FAILURE
            elif (not self.detected_cube) and self.mode == 1:
                return pt.common.Status.SUCCESS
            else:
                return pt.common.Status.SUCCESS
        
        elif not self.listened: 
            try:     
                rospy.loginfo("Checking if job is done!")
                rospy.sleep(5)
                detected_cube_pose_msg = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, timeout=30)
                rospy.loginfo("Detected cube pose on table 2 %s!", detected_cube_pose_msg)
                self.detected_cube = True
                
                #task_finished = True
                rospy.loginfo("Job is done, I nailed it!")
            except: 
                rospy.loginfo("Cube not detected on table 2!")
                rospy.loginfo("Job not done, damn it!")
            
            self.listened = True
            # tell the tree you're running
            return pt.common.Status.RUNNING

class resetpose(pt.behaviour.Behaviour):

    """
    Resets the pose of the robot and cube.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    mode = 0 (default); resets the robot and the cube to initial state (Task C)
    mode = 1          ; resets the cube only to initial state (Task A)
    """

    def __init__(self, mode = 0):
        
        # mode 
        self.mode = mode
        rospy.loginfo("Initialising the resetting of the pose of the robot and cube.")

        rospy.wait_for_service('/gazebo/set_model_state', timeout=30)
        self.reset_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)  # Create a service proxy

        self.robot_state = self.get_robot_state()
        self.cube_state = self.get_cube_state()

        # execution checker
        self.tried = False
        self.done = False

        self.success_robot = True 

        # become a behaviour
        super(resetpose, self).__init__("Reset cube and/or robot!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            self.reset_cube_req = self.reset_srv(self.cube_state)
            self.tried = True

            # command
            if self.mode == 0:
                self.reset_robot_req = self.reset_srv(self.robot_state)
                self.success_robot = self.reset_robot_req.success

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.success_robot and self.reset_cube_req.success:
            rospy.loginfo("Resetting of the pose of the robot and cube succeeded!.")
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.success_robot or not self.reset_cube_req.success:   
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

    def get_robot_state(self):
        robot_state = ModelState()
        robot_state.model_name = 'tiago'
        robot_state.pose.position.x = -1.146
        robot_state.pose.position.y = -6.155
        robot_state.pose.position.z = -0.001
        robot_state.pose.orientation.x = 0
        robot_state.pose.orientation.y = 0
        robot_state.pose.orientation.z = -0.7149132
        robot_state.pose.orientation.w = 0.6992132
        robot_state.twist.linear.x = 0
        robot_state.twist.linear.y = 0
        robot_state.twist.linear.z = 0
        robot_state.twist.angular.x = 0
        robot_state.twist.angular.y = 0
        robot_state.twist.angular.z = 0
        robot_state.reference_frame = 'map'
        return robot_state
    
    def get_cube_state(self ):
        cube_state = ModelState()
        cube_state.model_name = 'aruco_cube'
        cube_state.pose.position.x = -1.130530
        cube_state.pose.position.y = -6.653650
        cube_state.pose.position.z = 0.86250
        cube_state.pose.orientation.x = 0
        cube_state.pose.orientation.y = 0
        cube_state.pose.orientation.z = 0
        cube_state.pose.orientation.w = 1
        cube_state.twist.linear.x = 0
        cube_state.twist.linear.y = 0
        cube_state.twist.linear.z = 0
        cube_state.twist.angular.x = 0
        cube_state.twist.angular.y = 0
        cube_state.twist.angular.z = 0
        cube_state.reference_frame = 'map'
        return cube_state


class amcl_convergence_checker(pt.behaviour.Behaviour):

    """
    Checks if AMCL has converged by thresholding the standard 
    deviation in both x and y direction.
    """

    def __init__(self):

        rospy.loginfo("Initialising AMCL convergence checker.")

        self.particlecloud_top = '/particlecloud'

        self.std_position_threshold = 0.05

        self.init_localization = False

        # servers to the clear costmaps
        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.wait_for_service('/move_base/clear_costmaps', timeout=30)

        # become a behaviour
        super(amcl_convergence_checker, self).__init__("Localization converged?")

    def update(self):

        # Has the robot succeeded in localizing itself initially? 
        if self.init_localization:
            return pt.common.Status.SUCCESS

        particles = rospy.wait_for_message(self.particlecloud_top, PoseArray, timeout=5)
        #rospy.loginfo("Cloud particles received, checking convergence!")

        # Extract particles from the PoseArray message
        rec_particles = [(pose.position.x, pose.position.y) for pose in particles.poses]

        if len(rec_particles) > 0:
            self.check_convergence(rec_particles)

        # Localization succeeded ?
        if self.init_localization:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


    def check_convergence(self, particles):
        # Convert the list of particles to a numpy array for easier processing
        particles_array = np.array(particles)

        # Calculate the mean and standard deviation of the particle positions (x, y)
        x_stddev = np.std(particles_array[:, 0])  # Standard deviation of x
        y_stddev = np.std(particles_array[:, 1])  # Standard deviation of y

        # Tuning the standard deviation
        #rospy.loginfo(f"Standard deviations - X: {x_stddev}, Y: {y_stddev}")

        # Check if both x and y standard deviations are below the threshold
        if (x_stddev < self.std_position_threshold and y_stddev < self.std_position_threshold):
            rospy.loginfo(f"Standard deviations - X: {x_stddev}, Y: {y_stddev}")
            rospy.loginfo("Initial localization is successful. PF has converged!")
            self.init_localization = True
            # Clear costmaps
            clear_costmaps_req = self.clear_costmaps_srv()
            rospy.loginfo("Costmaps cleared successfully!")
        #else:
            #rospy.loginfo(f"Standard deviations - X: {x_stddev}, Y: {y_stddev}")
            #rospy.loginfo("Initial localization is not successful yet.")


class update_localization(pt.behaviour.Behaviour):

    """
    Completes an update step for the amcl
    """

    def __init__(self, mode = 0):

        rospy.loginfo("Initialising global and update localization.")

        self.mode = mode

        # servers to initiate global localization and update localization
        self.global_localization_srv = rospy.ServiceProxy('/global_localization', Empty)
        rospy.wait_for_service('/global_localization', timeout=30)
        
        if self.mode == 0:
            global_localization_req = self.global_localization_srv() # initialize global localization
            rospy.loginfo("Global localization initialized successfully!")
        elif self.mode == 1:
            self.global_localization_done = False

        self.update_localization_srv = rospy.ServiceProxy('/request_nomotion_update', Empty)
        rospy.wait_for_service('/request_nomotion_update', timeout=30)

        # To rotate when relocalize
        self.cmd_vel_top = "/key_vel"
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.move_msg = Twist()
        self.move_msg.linear.x = 0
        self.move_msg.angular.z = 1.5

        # become a behaviour
        super(update_localization, self).__init__("Update Robot Localization!")

    def update(self):
        
        if self.mode == 1 and (not self.global_localization_done):
            global_localization_req = self.global_localization_srv() # initialize global localization
            rospy.loginfo("Global localization initialized successfully!")
            self.global_localization_done = True

        self.update_localization_srv_req = self.update_localization_srv()

        if self.mode == 1:
            # Rotate
            rate = rospy.Rate(20)
            self.cmd_vel_pub.publish(self.move_msg)
            rate.sleep()

        #rospy.loginfo("Localization succeeded in updating!")
        return pt.common.Status.RUNNING

class cancel_goal(pt.behaviour.Behaviour):
    """
    Cancels goal
    """
    def __init__(self, ac_client):

        rospy.loginfo("Initialising cancel goal.")

        self.ac_client = ac_client

        self.goal_cancelled = False
        # become a behaviour
        super(cancel_goal, self).__init__("Cancel Goal!")

    def update(self):
        
        if not self.goal_cancelled:
            self.ac_client.cancel_goal()
            rospy.loginfo("Goal cancelled successfully")
            self.goal_cancelled = True

        return pt.common.Status.SUCCESS


class kidnap_checker(pt.behaviour.Behaviour):

    """
    Checks if robot has been kiddnapped!
    """

    def __init__(self, sendgoal):

        rospy.loginfo("Initialising kindap checker.")

        self.particlecloud_top = '/particlecloud'

        self.amcl_pose_top = '/amcl_pose'

        self.std_position_threshold = 0.4

        self.last_position_and_yaw = None
        #self.det_cov = 0.0

        self.kidnapped = False

        self.sendgoal = sendgoal

        # become a behaviour
        super(kidnap_checker, self).__init__("Kidnap checker ?")

    def update(self):

        # Is the robot kidnapped? 
        if self.kidnapped:
            return pt.common.Status.SUCCESS
        elif self.sendgoal.finished:
            return pt.common.Status.FAILURE

        particles = rospy.wait_for_message(self.particlecloud_top, PoseArray, timeout=5)
        #rospy.loginfo("Cloud particles received, checking convergence!")
        # Extract particles from the PoseArray message
        rec_particles = [(pose.position.x, pose.position.y) for pose in particles.poses]

        # Obtain pose estimate with covariance matrix
        pose_estimate = rospy.wait_for_message(self.amcl_pose_top, PoseWithCovarianceStamped, timeout=5)

        Cov_mat = pose_estimate.pose.covariance
        x_cov = Cov_mat[0]    
        y_cov = Cov_mat[7]    
        yaw_cov = Cov_mat[35]
        new_position_and_yaw = x_cov + y_cov + yaw_cov

        if self.last_position_and_yaw is None:
            self.last_position_and_yaw = new_position_and_yaw
            rospy.loginfo(f'Initial covariance {new_position_and_yaw}')
            return pt.common.Status.RUNNING
        
        if len(rec_particles) > 0:
            self.check_convergence(rec_particles, new_position_and_yaw)
            self.last_position_and_yaw = new_position_and_yaw


        if self.kidnapped:
            return pt.common.Status.SUCCESS 

        # tell the tree you've failed, you sucker!!
        return pt.common.Status.FAILURE
            
    def check_convergence(self, particles, new_position_and_yaw):
        # Convert the list of particles to a numpy array for easier processing
        particles_array = np.array(particles)

        # Calculate the mean and standard deviation of the particle positions (x, y)
        x_stddev = np.std(particles_array[:, 0])  # Standard deviation of x
        y_stddev = np.std(particles_array[:, 1])  # Standard deviation of y

        # Tuning the standard deviation
        #rospy.loginfo(f"Standard deviations - X + Y: {x_stddev + y_stddev}")

        change_in_position_and_yaw = new_position_and_yaw - self.last_position_and_yaw

        if abs(change_in_position_and_yaw) > 1e-2:
            rospy.loginfo(f"Change in P_xx*P_yy*P_yawyaw: {change_in_position_and_yaw}")
            rospy.loginfo('Robot kidnapped!!!!')
            self.kidnapped = True
            self.sendgoal.finished = True

        elif (x_stddev + y_stddev > self.std_position_threshold):
            #rospy.loginfo(f"Standard deviations - X + Y: {x_stddev + y_stddev}")
            rospy.loginfo("Robot is kidnapped, should cancel goal now!")
            self.kidnapped = True
            self.sendgoal.finished = True

    
    def get_kidnapped_state(self):
        return self.kidnapped


class sendgoal(pt.behaviour.Behaviour):

    """
    Sends a goal to the play motion action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    
    @param operation: The operation to perform (e.g., pick, place, cancel).
    """

    def __init__(self, operation, client=None):
        rospy.loginfo("Initializing send goal behaviour for " + operation + " operation!")
        
        # Set up action client
        if client:
            self.move_base_ac = client
        else:
            self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)

        # Connect to the action server
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server")
            exit()
        rospy.loginfo("%s: Connected to move_base action server")

        if operation =="cancel":
            self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
            self.cancel_msg = GoalID()
        else:
            # Get the pose from the pose topic
            operation_pose_top = rospy.get_param(rospy.get_name() + '/' + operation + '_pose_topic')
            self.operation_pose = rospy.wait_for_message(operation_pose_top, PoseStamped, timeout=5)
            
            # Get the goal from the pose topic
            self.goal = MoveBaseGoal()
            self.goal.target_pose = self.operation_pose
            rospy.loginfo("Received goal pose for " + operation + " operation!")

        # Execution checker, Boolean to check the task status
        self.operation = operation
        self.sent_goal = False
        self.finished = False

        # Become a behaviour
        super(sendgoal, self).__init__("Send goal:" + operation)

    def get_action_client(self):
        return self.move_base_ac

    def update(self):
        # Already done the task
        if self.finished:
            return pt.common.Status.SUCCESS
        
        # Not sent the goal yet
        elif not self.sent_goal and self.operation == "cancel":
            self.cancel_pub.publish(self.cancel_msg)
            self.sent_goal = True
            self.finished = True
            return pt.common.Status.SUCCESS

        # Not sent the goal yet
        elif not self.sent_goal and self.operation != "cancel":
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base_ac.send_goal(self.goal)
            self.sent_goal = True
            return pt.common.Status.RUNNING

        elif self.move_base_ac.get_state() == 0 or self.move_base_ac.get_state() == 1:
            return pt.common.Status.RUNNING 

        # Task is successful
        elif self.move_base_ac.get_result():
            self.finished = True
            return pt.common.Status.SUCCESS

        # Failed
        elif not self.move_base_ac.get_result():
            return pt.common.Status.FAILURE

        # Already sent the goal and not yet received the result
        else:
            return pt.common.Status.RUNNING


# class gather_cues(pt.behaviour.Behaviour):

#     """
#     Rotate to gather cues.
#     """

#     def __init__(self, angular, count_max):

#         rospy.loginfo("Initialising gather cues behaviour.")

#         # action space
#         #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
#         self.cmd_vel_top = "/key_vel"
#         #rospy.loginfo(self.cmd_vel_top)
#         self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

#         # command
#         self.move_msg = Twist()
#         self.move_msg.angular.z = angular

#         self.count_max = count_max
#         self.counter = 0

#         self.sent_message = False

#         # become a behaviour
#         super(gather_cues, self).__init__("Gather cues")

#     def update(self):
        
#         if self.counter == 0:
#             rospy.loginfo("Rotating to gather cues!")
#         if self.counter >= self.count_max:
#             if not self.sent_message:
#                 rospy.loginfo('Cues Gathered')
#                 self.sent_message = True
#             return pt.common.Status.SUCCESS
#         # send the message
#         rate = rospy.Rate(10)
#         self.cmd_vel_pub.publish(self.move_msg)
#         rate.sleep()
#         self.counter += 1

#         # tell the tree that you're running
#         return pt.common.Status.RUNNING 



# class mission_done(pt.behaviour.Behaviour):

#     """
#     Checks if mission is done.
#     """
#     def __init__(self):
#         rospy.loginfo("Initialising mission done behaviour.")

#         #self.task_finished = False
#         global task_finished
#         self.message_sent = False

#         # become a behaviour
#         super(mission_done, self).__init__("Mission done?")

#     def update(self):
#         if task_finished:
#             if not self.message_sent:
#                 rospy.loginfo('Mission DONE, I will do nothing now!')
#                 self.message_sent = True
#             return pt.common.Status.SUCCESS
#         else:
#             return pt.common.Status.FAILURE



class reset_tree_states(pt.behaviour.Behaviour):

    """
    Resets the tree states such that another iteration can follow.
    """
    def __init__(self, kidnap_checker, cancel_goal, update_localization, convergence_checker, send_goal_first, send_goal_kid,
                            detect_cube, pick_cube, moveto_place, place_cube, move_head, tuck_arm, resetpose):
        rospy.loginfo("Initialising reset tree states behaviour.")

        self.kidnap_checker = kidnap_checker  
        self.cancel_goal = cancel_goal
        self.update_localization = update_localization
        self.convergence_checker = convergence_checker
        self.send_goal_first = send_goal_first
        self.send_goal_kid = send_goal_kid
        self.detect_cube = detect_cube
        self.pick_cube = pick_cube
        self.moveto_place = moveto_place
        self.place_cube = place_cube
        self.move_head = move_head
        self.tuck_arm = tuck_arm
        self.resetpose = resetpose

        # become a behaviour
        super(reset_tree_states, self).__init__("Reset tree states!")

    def update(self):
        rospy.loginfo('Resetting all variables for another run')
        # Reset kidnap checker
        self.kidnap_checker.kidnapped = False

        # Reset cancel goal
        self.cancel_goal.goal_cancelled = False

        # Reset update localization
        self.convergence_checker.init_localization = False

        # Reset global localization
        self.update_localization.global_localization_done = False

        # Reset send goal first
        self.send_goal_first.sent_goal = False
        self.send_goal_first.finished = False

        # Reset send goal kidnap
        self.send_goal_kid.sent_goal = False
        self.send_goal_kid.finished = False

        # Reset pick cube
        self.pick_cube.tried = False
        self.pick_cube.done = False

        # Reset navigate to place
        self.moveto_place.sent_goal = False
        self.moveto_place.finished = False

        # Reset place cube
        self.place_cube.tried = False
        self.place_cube.done = False

        # Reset move head up
        self.move_head.tried = False
        self.move_head.done = False

        # Reset tuck arm
        self.tuck_arm.sent_goal = False
        self.tuck_arm.finished = False

        # Reset move head up
        self.resetpose.tried = False
        self.resetpose.done = False

        return pt.common.Status.FAILURE

# class test_case(pt.behaviour.Behaviour):

#     """
#     Rotate to gather cues.
#     """

#     def __init__(self, mission_done):

#         global task_finished

#         rospy.loginfo("Initialising test_case behaviour.")

#         # become a behaviour
#         super(test_case, self).__init__("Test case")

#     def update(self):
#         rospy.loginfo('Test case entered')
#         task_finished = True
#         # tell the tree that you're running
#         return pt.common.Status.SUCCESS 


# class return_true(pt.behaviour.Behaviour):

#     """
#     Rotate to gather cues.
#     """

#     def __init__(self ):

#         rospy.loginfo("Initialising return true behaviour.")

#         # become a behaviour
#         super(return_true, self).__init__("Return True")

#     def update(self):
#         return pt.common.Status.SUCCESS


# class do_nothing(pt.behaviour.Behaviour):

#     """
#     Rotate to gather cues.
#     """

#     def __init__(self ):

#         rospy.loginfo("Initialising do noting behaviour.")

#         # become a behaviour
#         super(do_nothing, self).__init__("Do nothing")

#     def update(self):
#         return pt.common.Status.SUCCESS '



# class improv_cube_detected_on_table2(pt.behaviour.Behaviour):

#     """
#     Returns if cube is placed on table 2.
#     """

#     def __init__(self, mode = 0):

#         rospy.loginfo("Initialising improved placement detection of cube on table 2.")

#         global task_finished

#         self.gazebo_model_states = '/gazebo/model_states'

#         self.detected_cube = False
#         self.listened = False

#         #self.mission_done = mission_done
#         self.mode = mode

#         # become a behaviour
#         super(improv_cube_detected_on_table2, self).__init__("Is_job_done")

#     def update(self):

#         # If listened return result
#         if self.listened:
#             if self.detected_cube:
#                 return pt.common.Status.SUCCESS
#             elif (not self.detected_cube) and self.mode == 0:
#                 return pt.common.Status.FAILURE
#             elif (not self.detected_cube) and self.mode == 1:
#                 return pt.common.Status.SUCCESS
#             else:
#                 return pt.common.Status.SUCCESS
        
#         elif not self.listened: 
#             rospy.loginfo("Checking if job is done!")
#             rospy.sleep(5)
#             detected_cube_pose_msg = rospy.wait_for_message(self.gazebo_model_states, ModelStates, timeout=30)

#             # Get the index of the 'aruco_cube' in the models list
#             index = detected_cube_pose_msg.name.index('aruco_cube')
            
#             # Extract pose and twist (velocity) information
#             aruco_pose = detected_cube_pose_msg.pose[index]
#             rospy.loginfo("Placed Aruco Cube Pose: \n%s", aruco_pose)
            
#             if self.check_cube_boundaries(aruco_pose):
#                 self.detected_cube = True
#                 task_finished = True
#                 rospy.loginfo("Job is done, I nailed it!")
#             else:
#                 rospy.loginfo("Cube not placed on table 2!")
#                 rospy.loginfo("Job not done, damn it!")
            
#             self.listened = True
#             # tell the tree you're running
#             return pt.common.Status.RUNNING


#     def check_cube_boundaries(self, aruco_pose):

#         if (3.09 <= aruco_pose.position.x and aruco_pose.position.x <= 3.83)  and (-2.302 <= aruco_pose.position.y and aruco_pose.position.y <= -1.357) and (0.6225 <= aruco_pose.position.z and aruco_pose.position.z <= 1.0625):
#             return True
#         else:
#             return False 



class mission_done(pt.behaviour.Behaviour):

    """
    Returns if cube is placed on table 2.
    """

    def __init__(self, mode = 0):

        rospy.loginfo("Initialising mission_done!")

        self.gazebo_model_states = '/gazebo/model_states'
        
        self.sent_message = False

        self.mode = mode

        # become a behaviour
        super(mission_done, self).__init__("missin_done")

    def update(self):

        #rospy.loginfo("Checking if job is done!")
        try: 
            detected_cube_pose_msg = rospy.wait_for_message(self.gazebo_model_states, ModelStates, timeout=30)
        except:
            return pt.common.Status.FAILURE

        # Get the index of the 'aruco_cube' in the models list
        index = detected_cube_pose_msg.name.index('aruco_cube')
        
        # Extract pose and twist (velocity) information
        aruco_pose = detected_cube_pose_msg.pose[index]
        #rospy.loginfo("Placed Aruco Cube Pose: \n%s", aruco_pose)
        
        if self.check_cube_boundaries(aruco_pose):
            if not self.sent_message:
                rospy.loginfo("Job is done, I nailed it!")
                self.sent_message = True
            return pt.common.Status.SUCCESS
        else:
            if self.mode == 0:
                if not self.sent_message:
                    rospy.loginfo("Cube not placed on table 2!")
                    rospy.loginfo("Job not done, damn it!")
                    self.sent_message = True
            return pt.common.Status.FAILURE
        
        # tell the tree you're running
        return pt.common.Status.RUNNING


    def check_cube_boundaries(self, aruco_pose):

        if self.mode == 1:
            if ((3.09 <= aruco_pose.position.x and aruco_pose.position.x <= 3.83)  and (-2.302 <= aruco_pose.position.y and aruco_pose.position.y <= -1.357) and (0.6225 <= aruco_pose.position.z and aruco_pose.position.z <= 1.0625)):
                return True
            else:
                return False 

        if self.mode == 0:
            if ((-1.44 <= aruco_pose.position.x and aruco_pose.position.x <= -0.71)  and (-4.85 <= aruco_pose.position.y and aruco_pose.position.y <= -3.9) and (0.6225 <= aruco_pose.position.z and aruco_pose.position.z <= 1.0625)):
                return True
            else:
                return False 



# # Christopher Iliffe Sprague
# # sprague@kth.se
# # Behaviours needed for the example student solution.

# import numpy as np
# import py_trees as pt, py_trees_ros as ptr, rospy
# from geometry_msgs.msg import Twist
# from actionlib import SimpleActionClient, GoalStatus
# from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
# from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
# from std_srvs.srv import Empty, SetBool, SetBoolRequest  
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
# from gazebo_msgs.msg import ModelState 
# from gazebo_msgs.srv import SetModelState
# from geometry_msgs.msg import PoseArray

# class Blackboard(pt.behaviour.Behaviour):
    
#     """
#     A blackboard to store the state of the stages, shared among all nodes.
#     """

#     def __init__(self):
#         rospy.loginfo("Initialize the stage manager.")

#         # Initialize all stages with False
#         self.stages = {

#             # Localization param
#             "initialised":    False,  # The robot has been initialised
#             "localizing":     False,  # The robot is localizing

#             # State param
#             "converged":      False,  # AMCL has converged, the robot know where it is precisely
#             "safe":           False,  # The robot is in a safe position, may achieve this zone when the robot is moving
#             "kidnapped":      True,   # The robot has been kidnapped and doesn't know where it is

#             # Task param
#             "pick_pose":      False,
#             "detect_cube":    False,
#             "pick":           False,
#             "place_pose":     False,
#             "place":          False,
            
#             # Final
#             "cube_on_table2": False,
#         }

#     def set(self, stage, status):
#         if stage in self.stages:
#             self.stages[stage] = status
#         else:
#             rospy.logerr("Invalid stage!")
    
#     def get(self, stage):
#         if stage in self.stages:
#             return self.stages[stage]
#         else:
#             rospy.logerr("Invalid stage!")

# class check_convergence(pt.behaviour.Behaviour):

#     """
#     Checks if AMCL has converged by thresholding the standard 
#     deviation in both x and y direction.
#     """

#     def __init__(self, blackboard):

#         rospy.loginfo("Initialize AMCL convergence checker.")

#         self.blackboard = blackboard

#         # Topics
#         self.particlecloud_top = '/particlecloud'
#         self.amcl_pose_top = '/amcl_pose'

#         # Thresholds
#         self.safe_threshold = 0.5
#         self.converged_threshold = 0.1
#         self.last_sum_cov = 0

#         # Subscriber
#         self.pose_subscriber = rospy.Subscriber(self.amcl_pose_top, PoseWithCovarianceStamped, self.pose_callback)

#         # Servers to the clear costmaps
#         self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
#         rospy.wait_for_service('/move_base/clear_costmaps', timeout=30)

#         # Become a behaviour
#         super(check_convergence, self).__init__("Convergence checker")
    
#     def update(self):

#         x_stddev, y_stddev = self.get_stddev()
#         kidnapped = self.check_kidnap()

#         if x_stddev is None:
#             return pt.common.Status.FAILURE

#         # Not converged and not safe
#         if (x_stddev > self.safe_threshold or y_stddev > self.safe_threshold) or kidnapped:
#             if self.blackboard.get("localizing"):
#                 print_status = "localizing, kidnapped range"
#             else:
#                 self.blackboard.set("converged", False)
#                 self.blackboard.set("safe", False)
#                 self.blackboard.set("kidnapped", True)
#                 print_status = "kidnapped"
#                 rospy.loginfo("Robot is kidnapped! Start relocalize")

#         # Converged and safe
#         elif (x_stddev < self.converged_threshold and y_stddev < self.converged_threshold):
            
#             # End of localizing
#             if self.blackboard.get("localizing"):
#                 self.blackboard.set("localizing", False)
#                 self.blackboard.set("initialised", True)
#                 clear_costmaps_req = self.clear_costmaps_srv()
#                 rospy.loginfo("Robot is converged and safe now! End localize")

#             self.blackboard.set("converged", True)
#             self.blackboard.set("safe", True)
#             self.blackboard.set("kidnapped", False)
#             print_status = "converged"
        
#         # Not converged but safe
#         else:
#             if self.blackboard.get("localizing"):
#                 print_status = "localizing , safe range"
#             else:
#                 self.blackboard.set("converged", False)
#                 self.blackboard.set("safe", True)
#                 self.blackboard.set("kidnapped", False)
#                 print_status = "safe"
        
#         # rospy.loginfo(f"Standard deviations - X: {x_stddev}, Y: {y_stddev}")
#         # rospy.loginfo(f"Robot is {print_status}!")

#         return pt.common.Status.SUCCESS
    
#     def get_stddev(self):
#         particles = rospy.wait_for_message(self.particlecloud_top, PoseArray, timeout=5)
#         rec_particles = [(pose.position.x, pose.position.y) for pose in particles.poses]
        
#         if len(rec_particles) == 0:
#             rospy.loginfo("No particles received, cannot check convergence!")
#             return None, None
        
#         particles_array = np.array(rec_particles)
#         x_stddev = np.std(particles_array[:, 0])
#         y_stddev = np.std(particles_array[:, 1])

#         return x_stddev, y_stddev
    
#     def check_kidnap(self):
#         cov_mat = self.pose_estimate.pose.covariance
#         x_cov = cov_mat[0]    
#         y_cov = cov_mat[7]    
#         yaw_cov = cov_mat[35]
#         sum_cov = x_cov + y_cov + yaw_cov
#         delta_cov = sum_cov - self.last_sum_cov
#         self.last_sum_cov = sum_cov

#         # print(delta_cov)
#         if abs(delta_cov) > 0.02:
#             return True
#         return False
    
#     def pose_callback(self, msg):
#         self.pose_estimate = msg

# class update_localization(pt.behaviour.Behaviour):

#     """
#     Completes an update step for the amcl
#     """

#     def __init__(self, blackboard):

#         rospy.loginfo("Initialize global and update localization.")

#         self.blackboard = blackboard

#         # Global localization service
#         self.global_localization_srv = rospy.ServiceProxy('/global_localization', Empty)
#         rospy.wait_for_service('/global_localization', timeout=30)

#         # Update localization service
#         self.update_localization_srv = rospy.ServiceProxy('/request_nomotion_update', Empty)
#         rospy.wait_for_service('/request_nomotion_update', timeout=30)

#         # To rotate when relocalize
#         self.cmd_vel_top = "/key_vel"
#         self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
#         self.move_msg = Twist()
#         self.move_msg.linear.x = 0
#         self.move_msg.angular.z = 0.2

#         # become a behaviour
#         super(update_localization, self).__init__("Update Robot Localization!")

#     def update(self):

#         # Get the state of the robot from blackboard
#         initialised = self.blackboard.get("initialised")
#         localizing = self.blackboard.get("localizing")
#         converged = self.blackboard.get("converged")
#         safe = self.blackboard.get("safe")
#         kidnapped = self.blackboard.get("kidnapped")

#         if localizing:
#             # rate = rospy.Rate(20)
#             self.cmd_vel_pub.publish(self.move_msg)
#             # rate.sleep()
#             self.update_localization_srv_req = self.update_localization_srv()
#             status = pt.common.Status.RUNNING
#             # rospy.loginfo("Localizing")

#         elif not initialised:
#             rospy.loginfo("Robot not initialised yet! Start localize.")
#             global_localization_req = self.global_localization_srv()
#             self.blackboard.set("localizing", True)
#             status = pt.common.Status.SUCCESS
        
#         elif kidnapped:
#             # rospy.loginfo("Robot kidnapped, should cancel goal now!")
#             self.blackboard.set("initialised", False)
#             status = pt.common.Status.SUCCESS
        
#         elif not converged and safe:
#             # rospy.loginfo("Robot not converged, but safe!")
#             # self.update_localization_srv_req = self.update_localization_srv()
#             status = pt.common.Status.SUCCESS

#         else:
#             # converged and safe
#             # rospy.loginfo("Robot is converged and safe!")
#             # self.update_localization_srv_req = self.update_localization_srv()
#             status = pt.common.Status.SUCCESS
        
#         return status

# class move_pose(pt.behaviour.Behaviour):

#     """
#     Sends a goal to the play motion action server.
#     Returns running whilst awaiting the result,
#     success if the action was succesful, and v.v..
#     """

#     def __init__(self, operation, blackboard):
#         rospy.loginfo("Initializing move pose behaviour.")
        
#         # Execution checker, Boolean to check the task status
#         self.blackboard = blackboard
#         self.operation = operation
#         self.tried = False   

#         # Set up action client
#         self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)

#         # Connect to the action server
#         if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
#             rospy.logerr("Could not connect to /move_base action server")
#             exit()
#         rospy.loginfo("Connected to move_base action server")

#         # Get the pose from the pose topic
#         operation_pose_top = rospy.get_param(rospy.get_name() + '/' + operation + '_topic')
#         self.operation_pose = rospy.wait_for_message(operation_pose_top, PoseStamped, timeout=5)

#         # Get the goal from the pose topic
#         self.goal_data = MoveBaseGoal()
#         self.goal_data.target_pose = self.operation_pose
#         self.goal_data.target_pose.header.frame_id = "map"
#         rospy.loginfo("Received goal pose for " + operation + " operation!")

#         # Become a behaviour
#         super(move_pose, self).__init__(operation)

#     def get_goal(self):
#         """
#         Goal sequence:
#             "pick_pose"  
#             "detect_cube"
#             "pick"      
#             "place_pose"
#             "place"     
#         """
#         if self.blackboard.get("kidnapped"):
#             return "cancel"
#         elif self.blackboard.get("localizing"):
#             return "later"
#         elif not self.blackboard.get("pick_pose"):
#             return "pick_pose"
#         elif not self.blackboard.get("detect_cube"):
#             return "detect_cube"
#         elif not self.blackboard.get("pick"):
#             return "pick"
#         elif not self.blackboard.get("place_pose"):
#             return "place_pose"
#         elif not self.blackboard.get("place"):
#             return "place"

#     def update(self):
#         self.goal = self.get_goal()

#         # Send by other nodes
#         exclude_goals = ["detect_cube", "pick", "place"]

#         # Cancel the goal when kidnapped
#         if self.blackboard.get("kidnapped") and self.blackboard.get("initialised"):
#             if self.tried:
#                 self.move_base_ac.cancel_goal()
#                 rospy.loginfo("Goal cancelled successfully")
#             status = pt.common.Status.SUCCESS
        
#         # The robot is localizing or not yet initialised
#         elif self.blackboard.get("localizing") or not self.blackboard.get("initialised"):
#             status = pt.common.Status.FAILURE

#         # Skip the goal if operation not matching with the goal
#         elif self.goal in exclude_goals or self.operation != self.goal:
#             # rospy.loginfo(self.operation + " not match goal: " + self.goal)
#             status = pt.common.Status.SUCCESS

#         # Skip the goal if already done
#         elif self.blackboard.get(self.operation):
#             # rospy.loginfo(self.operation + " already done")
#             status = pt.common.Status.SUCCESS

#         # Operation matched with the goal and not yet tried
#         elif self.operation == self.goal and not self.blackboard.get(self.operation) and not self.tried:
#             rospy.loginfo("Doing " + self.operation)
#             self.goal_data.target_pose.header.stamp = rospy.Time.now()
#             self.move_base_ac.send_goal(self.goal_data)
#             self.tried = True
#             status = pt.common.Status.RUNNING

#         # Already sent the goal and not yet received the result
#         elif self.tried:
#             # self.move_base_ac.wait_for_result()
#             state = self.move_base_ac.get_state()

#             if state == GoalStatus.ABORTED or state == GoalStatus.REJECTED:
#                 rospy.loginfo(self.operation + "Failed!!")
#                 # May because the robot has the wrong localization
#                 # self.blackboard.set("kidnapped", True)
#                 self.blackboard.set("initialised", False)
#                 self.tried = False
#                 status = pt.common.Status.FAILURE
#             elif state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
#                 status = pt.common.Status.RUNNING
#             elif state == GoalStatus.SUCCEEDED: # TODO: Check if this works
#                 self.blackboard.set(self.operation, True)
#                 status = pt.common.Status.SUCCESS
#                 rospy.loginfo(self.operation + "Done!")
#             else:
#                 status = pt.common.Status.RUNNING

#         # Task is successful
#         elif self.move_base_ac.get_result():
#             self.blackboard.set(self.operation, True)
#             status = pt.common.Status.SUCCESS

#         return status

# class move_cube(pt.behaviour.Behaviour):

#     """
#     Pick or place the cube.
#     Returns running whilst awaiting the result,
#     success if the action was succesful, and v.v..
#     """

#     def __init__(self, operation, blackboard):

#         rospy.loginfo("Initializing move cube behaviour.")

#         # Execution checker
#         self.blackboard = blackboard
#         self.operation = operation
#         self.tried = False

#         # Setup the server
#         mv_cube_srv = '/' + operation + '_srv'
#         mv_cube_srv_nm = rospy.get_param(rospy.get_name() + mv_cube_srv)
#         self.move_cube_srv = rospy.ServiceProxy(mv_cube_srv_nm, SetBool)
#         rospy.wait_for_service(mv_cube_srv_nm, timeout=30)

#         # become a behaviour
#         super(move_cube, self).__init__("GET EM!")

#     def get_goal(self):
#         """
#         Goal sequence:
#             "pick_pose"  
#             "detect_cube"
#             "pick"      
#             "place_pose"
#             "place"     
#         """
#         if self.blackboard.get("kidnapped"):
#             return "cancel"
#         elif not self.blackboard.get("pick_pose"):
#             return "pick"
#         elif not self.blackboard.get("detect_cube"):
#             return "detect_cube"
#         elif not self.blackboard.get("pick"):
#             return "pick"
#         elif not self.blackboard.get("place_pose"):
#             return "place_pose"
#         elif not self.blackboard.get("place"):
#             return "place"

#     def update(self):
#         self.goal = self.get_goal()

#         # Send by other nodes
#         # NOTE: we can't cancel the service here
#         exclude_goals = ["detect_cube", "pick_pose", "place_pose", "cancel"]

#         # Skip the goal if operation not matching with the goal
#         if self.goal in exclude_goals or self.operation != self.goal:
#             status = pt.common.Status.SUCCESS

#         # Skip the goal if already done
#         elif self.blackboard.get(self.operation):
#             status = pt.common.Status.SUCCESS

#         # Operation matched with the goal and not yet sent the goal
#         elif not self.tried:
#             self.move_cube_req = self.move_cube_srv()
#             self.tried = True
#             status = pt.common.Status.RUNNING

#         # Already sent the goal and not yet received the result
#         elif self.tried and self.move_cube_req.success:
#             self.blackboard.set(self.operation, True)
#             status = pt.common.Status.SUCCESS

#         # Task is failed
#         else:
#             status = pt.common.Status.FAILURE

#         return status
    
# class detect_cube(pt.behaviour.Behaviour):

#     """
#     Sends a goal to the play motion action server.
#     Returns running whilst awaiting the result,
#     success if the action was succesful, and v.v..
#     """

#     def __init__(self, blackboard):

#         rospy.loginfo("Initializing detect cube behaviour.")

#         # Execution checker
#         self.blackboard = blackboard
#         self.tried = False

#         # Set up action client
#         self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
#         if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
#             rospy.logerr("%s: Could not connect to /play_motion action server")
#             exit()
#         rospy.loginfo("%s: Connected to play_motion action server")

#         # Goal setup
#         self.goal = PlayMotionGoal()
#         self.goal.motion_name = 'inspect_surroundings'
#         self.goal.skip_planning = True

#         # become a behaviour
#         super(detect_cube, self).__init__("Detect cube!")

#     def update(self):
        
#         # Cancel the goal when kidnapped
#         if self.blackboard.get("kidnapped") and self.blackboard.get("initialised"):
#             self.play_motion_ac.cancel_goal()
#             rospy.loginfo("Goal cancelled successfully")
#             status = pt.common.Status.SUCCESS

#         # Skip the goal if already done
#         elif self.blackboard.get("detect_cube"):
#             status = pt.common.Status.SUCCESS
        
#         # Not yet sent the goal
#         elif not self.tried:
#             self.play_motion_ac.send_goal(self.goal)
#             self.tried = True
#             status = pt.common.Status.RUNNING

#         # Already sent the goal and not yet received the result
#         elif self.tried:
#             self.play_motion_ac.wait_for_result()
#             state = self.play_motion_ac.get_state()

#             if state == GoalStatus.ABORTED or state == GoalStatus.REJECTED:
#                 status = pt.common.Status.FAILURE
#             elif state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
#                 status = pt.common.Status.RUNNING
#             elif state == GoalStatus.SUCCEEDED: # TODO: Check if this works
#                 self.blackboard.set("detect_cube", True)
#                 status = pt.common.Status.SUCCESS
#             else:
#                 status = pt.common.Status.RUNNING

#         # Task is successful
#         elif self.move_base_ac.get_result():
#             self.blackboard.set("detect_cube", True)
#             status = pt.common.Status.SUCCESS

#         return status

# class check_end(pt.behaviour.Behaviour):    

#     """
#     Returns if the robot has completed the task.
#         - If done, return SUCCESS
#         - If not done, reset the task and return FAILURE
#     """

#     def __init__(self, blackboard):

#         rospy.loginfo("Initializing check end behaviour.")

#         # Execution checker
#         self.blackboard = blackboard

#         # Check cube position
#         self.model_state_top = '/gazebo/model_states'

#         # Set up reset service
#         rospy.wait_for_service('/gazebo/set_model_state', timeout=30)
#         self.reset_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

#         # Become a behaviour
#         super(check_end, self).__init__("Check end!")

#     def update(self):

#         # If cube is already detected on table 2, done
#         if self.blackboard.get("cube_on_table2"):
#             status = pt.common.Status.SUCCESS
        
#         # If kidnapped, not yet done
#         elif self.blackboard.get("kidnapped") or self.blackboard.get("localizing"):
#             status = pt.common.Status.FAILURE

#         # If the previous task not done
#         elif not self.blackboard.get("pick_pose") or not self.blackboard.get("detect_cube") or not self.blackboard.get("pick") \
#             or not self.blackboard.get("place_pose") or not self.blackboard.get("place"):

#             status = pt.common.Status.FAILURE
        
#         # Already placed the cube and under safe situation
#         else:
#             done = self.check_cube_pos()
#             if done:
#                 rospy.loginfo("Ive done everything")
#                 self.blackboard.set("cube_on_table2", True)
#                 status = pt.common.Status.SUCCESS
#             else :
#                 rospy.loginfo("Fuck im a failure, restart again")
#                 self.reset_cube_pos()
#                 self.blackboard.set("pick_pose", False)
#                 self.blackboard.set("detect_cube", False)
#                 self.blackboard.set("pick", False)
#                 self.blackboard.set("place_pose", False)
#                 self.blackboard.set("place", False)
#                 status = pt.common.Status.FAILURE

#         return status

#     def check_cube_pos(self):
#         cube_pos = rospy.wait_for_message(self.model_state_top, ModelStates, timeout=30)
#         cube_idx = cube_pos.name.index('aruco_cube')
#         aruco_pos = cube_pos.pose[index]

#         x_cube = aruco_pos.position.x
#         y_cube = aruco_pos.position.y
#         z_cube = aruco_pos.position.z

#         x_min = 3.09
#         x_max = 3.83
#         y_min = -2.3
#         y_max = -1.36
#         z_min = 0.62
#         z_max = 1.06

#         if x_cube > x_max or x_cube < x_min or \
#             y_cube > y_max or y_cube < y_min or \
#             z_cube > z_max or z_cube < z_min :
#             return False
#         else :
#             return True

#     def reset_cube_pos(self):
#         cube_state = ModelState()
#         cube_state.model_name = 'aruco_cube'
#         cube_state.pose.position.x = -1.130530
#         cube_state.pose.position.y = -6.653650
#         cube_state.pose.position.z = 0.86250
#         cube_state.pose.orientation.x = 0
#         cube_state.pose.orientation.y = 0
#         cube_state.pose.orientation.z = 0
#         cube_state.pose.orientation.w = 1
#         cube_state.twist.linear.x = 0
#         cube_state.twist.linear.y = 0
#         cube_state.twist.linear.z = 0
#         cube_state.twist.angular.x = 0
#         cube_state.twist.angular.y = 0
#         cube_state.twist.angular.z = 0
#         cube_state.reference_frame = 'map'
#         self.reset_srv(cube_state)



# class tuckarm(pt.behaviour.Behaviour):

#     """
#     Sends a goal to the tuck arm action server.
#     Returns running whilst awaiting the result,
#     success if the action was succesful, and v.v..
#     """

#     def __init__(self):

#         rospy.loginfo("Initialising tuck arm behaviour.")

#         # Set up action client
#         self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

#         # personal goal setting
#         self.goal = PlayMotionGoal()
#         self.goal.motion_name = 'home'
#         self.goal.skip_planning = True

#         # execution checker
#         self.sent_goal = False
#         self.finished = False

#         # become a behaviour
#         super(tuckarm, self).__init__("Tuck arm!")

#     def update(self):

#         # already tucked the arm
#         if self.finished: 
#             return pt.common.Status.SUCCESS
        
#         # command to tuck arm if haven't already
#         elif not self.sent_goal:

#             # send the goal
#             self.play_motion_ac.send_goal(self.goal)
#             self.sent_goal = True

#             # tell the tree you're running
#             return pt.common.Status.RUNNING

#         # if I was succesful! :)))))))))
#         elif self.play_motion_ac.get_result():

#             # than I'm finished!
#             self.finished = True
#             return pt.common.Status.SUCCESS

#         # if failed
#         elif not self.play_motion_ac.get_result():
#             return pt.common.Status.FAILURE

#         # if I'm still trying :|
#         else:
#             return pt.common.Status.RUNNING


# class movehead(pt.behaviour.Behaviour):

#     """
#     Lowers or raisesthe head of the robot.
#     Returns running whilst awaiting the result,
#     success if the action was succesful, and v.v..
#     """

#     def __init__(self, direction):

#         rospy.loginfo("Initialising move head behaviour.")

#         # server
#         mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
#         self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
#         rospy.wait_for_service(mv_head_srv_nm, timeout=30)

#         # head movement direction; "down" or "up"
#         self.direction = direction

#         # execution checker
#         self.tried = False
#         self.done = False

#         # become a behaviour
#         super(movehead, self).__init__("Lower head!")

#     def update(self):

#         # success if done
#         if self.done:
#             return pt.common.Status.SUCCESS

#         # try if not tried
#         elif not self.tried:

#             # command
#             self.move_head_req = self.move_head_srv(self.direction)
#             self.tried = True

#             # tell the tree you're running
#             return pt.common.Status.RUNNING

#         # if succesful
#         elif self.move_head_req.success:
#             self.done = True
#             return pt.common.Status.SUCCESS

#         # if failed
#         elif not self.move_head_req.success:
#             return pt.common.Status.FAILURE

#         # if still trying
#         else:
#             return pt.common.Status.RUNNING
