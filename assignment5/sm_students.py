#!/usr/bin/env python3

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    def __init__(self):
        
        self.node_name = "Student SM"

        # Access rosparams
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose')
        self.robot_base_frame = rospy.get_param(rospy.get_name() + '/robot_base_frame')
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        # Subscribe to topics

        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        rospy.wait_for_service(self.place_srv_nm, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.aruco_pose_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=10)

        # Set up action clients
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)


        # Init state machine
        self.state = 0
        rospy.sleep(10)
        self.check_states()


    def check_states(self):

        while not rospy.is_shutdown() and self.state != 5:
            
            # if self.state == 0:               
            #     rospy.loginfo("%s: Tucking the arm...", self.node_name)
            #     goal = PlayMotionGoal()
            #     goal.motion_name = 'home'
            #     goal.skip_planning = True
            #     self.play_motion_ac.send_goal(goal)
            #     success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))
            #     if success_tucking:
            #         rospy.loginfo("%s: Tuck: ", self.play_motion_ac.get_result())
            #         self.state = 10
            #     else:
            #         self.play_motion_ac.cancel_goal()
            #         rospy.logerr("%s: play_motion failed to inspect tuck, reset simulation", self.node_name)
            #         self.state = 5
            #     rospy.sleep(1)
            
            # # State 2: Picking the cube
            if self.state  == 0:
                # Step 1: Publishing cube's pose:
                cube_pose_msg = PoseStamped()
                # Fill in the pose of the cube position and orientation
                cube_pose_msg.pose.position.x = 0.5280793094447964
                cube_pose_msg.pose.position.y = 0.03512330540299729
                cube_pose_msg.pose.position.z = 0.8709514850523552
                cube_pose_msg.pose.orientation.x = 0.0
                cube_pose_msg.pose.orientation.y = 0.0
                cube_pose_msg.pose.orientation.z = 0.0
                cube_pose_msg.pose.orientation.w = 0.0
                rate = rospy.Rate(1)
                rospy.loginfo("%s: Publishing cube's pose...", self.node_name)
                # Set the header
                cube_pose_msg.header.frame_id = self.robot_base_frame
                cube_pose_msg.header.stamp = rospy.Time.now()
                self.aruco_pose_pub.publish(cube_pose_msg)
                rate.sleep()

                rospy.sleep(1)

                # Step 2: Call the pick_service
                try:
                    rospy.loginfo("%s: Picking the cube...", self.node_name)
                    pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
                    res_pick_srv = pick_srv()

                    if res_pick_srv.success == True:
                        self.state = 1
                        rospy.loginfo("%s: Picking the cube succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Picking the cube failed!", self.node_name)
                        self.state = 5

                    rospy.sleep(1)

                except rospy.ServiceException as e:
                    print("Service call to pick_gui server failed: %s"%e)

                
            # # State 1:  Move the robot "manually" to other table
            if self.state == 1:
                move_msg = Twist()
                move_msg.angular.z = -3

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                rospy.loginfo("%s: Moving towards table", self.node_name)
                while not rospy.is_shutdown() and cnt < 15:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                move_msg.linear.x = 0.5
                move_msg.angular.z = 0
                cnt = 0
                while not rospy.is_shutdown() and cnt < 19:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 2
                rospy.sleep(1)


            # State 2: Placing the cube
            if self.state == 2:
                # Call the place_service
                try:
                    rospy.loginfo("%s: Placing the cube...", self.node_name)
                    place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)
                    res_place_srv = place_srv()

                    if res_place_srv.success == True:
                        self.state = 4
                        rospy.loginfo("%s: Placing the cube succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Placing the cube failed!", self.node_name)
                        self.state = 5

                    rospy.sleep(1)

                except rospy.ServiceException as e:
                    print("Service call to place_gui server failed: %s"%e)

            # Error handling
            if self.state == 5:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return


# import py_trees as pt, py_trees_ros as ptr

# class BehaviourTree(ptr.trees.BehaviourTree):

# 	def __init__(self):

# 		rospy.loginfo("Initialising behaviour tree")

# 		# go to door until at door
# 		b0 = pt.composites.Selector(
# 			name="Go to door fallback", 
# 			children=[Counter(30, "At door?"), Go("Go to door!", 1, 0)]
# 		)

# 		# tuck the arm
# 		b1 = TuckArm()

# 		# go to table
# 		b2 = pt.composites.Selector(
# 			name="Go to table fallback",
# 			children=[Counter(5, "At table?"), Go("Go to table!", 0, -1)]
# 		)

# 		# move to chair
# 		b3 = pt.composites.Selector(
# 			name="Go to chair fallback",
# 			children=[Counter(13, "At chair?"), Go("Go to chair!", 1, 0)]
# 		)

# 		# lower head
# 		b4 = LowerHead()

# 		# become the tree
# 		tree = pt.composites.Sequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
# 		super(BehaviourTree, self).__init__(tree)

# 		# execute the behaviour tree
# 		self.setup(timeout=10000)
# 		while not rospy.is_shutdown(): self.tick_tock(1)


# class Counter(pt.behaviour.Behaviour):

# 	def __init__(self, n, name):

# 		# counter
# 		self.i = 0
# 		self.n = n

# 		# become a behaviour
# 		super(Counter, self).__init__(name)

# 	def update(self):

# 		# count until n
# 		while self.i <= self.n:

# 			# increment count
# 			self.i += 1

# 			# return failure :(
# 			return pt.common.Status.FAILURE

# 		# succeed after counter done :)
# 		return pt.common.Status.SUCCESS


# class Go(pt.behaviour.Behaviour):

# 	def __init__(self, name, linear, angular):

# 		# action space
# 		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
# 		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

# 		# command
# 		self.move_msg = Twist()
# 		self.move_msg.linear.x = linear
# 		self.move_msg.angular.z = angular

# 		# become a behaviour
# 		super(Go, self).__init__(name)

# 	def update(self):

# 		# send the message
# 		rate = rospy.Rate(10)
# 		self.cmd_vel_pub.publish(self.move_msg)
# 		rate.sleep()

# 		# tell the tree that you're running
# 		return pt.common.Status.RUNNING


# class TuckArm(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# Set up action client
# 		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

# 		# personal goal setting
# 		self.goal = PlayMotionGoal()
# 		self.goal.motion_name = 'home'
# 		self.goal.skip_planning = True

# 		# execution checker
# 		self.sent_goal = False
# 		self.finished = False

# 		# become a behaviour
# 		super(TuckArm, self).__init__("Tuck arm!")

# 	def update(self):

# 		# already tucked the arm
# 		if self.finished: 
# 			return pt.common.Status.SUCCESS
		
# 		# command to tuck arm if haven't already
# 		elif not self.sent_goal:

# 			# send the goal
# 			self.play_motion_ac.send_goal(self.goal)
# 			self.sent_goal = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# if I was succesful! :)))))))))
# 		elif self.play_motion_ac.get_result():

# 			# than I'm finished!
# 			self.finished = True
# 			return pt.common.Status.SUCCESS

# 		# if I'm still trying :|
# 		else:
# 			return pt.common.Status.RUNNING
		


# class LowerHead(pt.behaviour.Behaviour):

# 	def __init__(self):

# 		# server
# 		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
# 		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
# 		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

# 		# execution checker
# 		self.tried = False
# 		self.tucked = False

# 		# become a behaviour
# 		super(LowerHead, self).__init__("Lower head!")

# 	def update(self):

# 		# try to tuck head if haven't already
# 		if not self.tried:

# 			# command
# 			self.move_head_req = self.move_head_srv("down")
# 			self.tried = True

# 			# tell the tree you're running
# 			return pt.common.Status.RUNNING

# 		# react to outcome
# 		else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE


	

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		#StateMachine()
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
