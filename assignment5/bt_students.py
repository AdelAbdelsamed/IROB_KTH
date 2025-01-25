#!/usr/bin/env python3

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")
		# ##Task C:
		## tuck the arm
		# b0 = tuckarm()

		# # detect the cube
		# b1 = detectcube()

		# # pick the cube
		# b2 = movecube("pick")

		# # go to table
		# b3 = pt.composites.Selector(
		# 	name="Go to table fallback",
		# 	children=[counter(15, "At table?"), go("Go to table!", 0, -3)]
		# )

		# b4 = pt.composites.Selector(
		# 	name="Go to table fallback",
		# 	children=[counter(16, "At table?"), go("Go to table!", 0.5, 0)]
		# )

		# # place the cube on second table
		# b5 = movecube("place")

		# # Create sequence to execute if job is not done!
		# sequence = RSequence(name="Job not done and Reset", children=[tuckarm(), wait(10, "Resetting!"), resetpose()])
		
		# # checks if cube is placed on table 2 and job is done
		# b6 = pt.composites.Selector(
		# 	name="Check if detected",
		# 	children=[mission_done(), sequence]
		# )

		# # become the tree
		# tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, b5, b6])
		# super(BehaviourTree, self).__init__(tree)

		
		# TASK A ONE TIME
		# b0 = tuckarm()
		# b01 = movehead('up')

		# b1 = pt.composites.Selector(
		# 	name="Perform initial localization",
		# 	children=[amcl_convergence_checker(), update_localization(mode = 0)]
		# ) 

		# b212 =  pt.composites.Selector(
		# 	name="Perform re-localization",
		# 	children=[amcl_convergence_checker(), update_localization(mode = 1)]
		# ) 

		# send_goal_pick = sendgoal("pick")
		# b21 = RSequence(name="Kidnap checking sequence", children=[kidnap_checker(send_goal_pick), cancel_goal(send_goal_pick.get_action_client()), b212, sendgoal("pick")]) # send goal still missing
		# b22 = send_goal_pick 

		# b2 = pt.composites.Selector(
		# 	name="Nanvigation and kidnap checking",
		# 	children=[b21, b22]
		# )

		# b3 = detectcube()
		
		# b4 = movecube('pick')

		# b5 = sendgoal('place')

		# b6 = movecube('place')

		# b7 = cube_detected_on_table2()

		# tree = RSequence(name="Main sequence", children=[b0,b01, b1, b2, b3, b4, b5, b6, b7])


		b0 = tuckarm()

		b1 = movehead('up')

		b2 = pt.composites.Selector(
			name="Perform initial localization",
			children=[amcl_convergence_checker(), update_localization(mode = 0)]
		) 

		upd_loc = update_localization(mode = 1)
		conv_checker = amcl_convergence_checker()
		b413 =  pt.composites.Selector(
			name="Perform re-localization",
			children=[conv_checker, upd_loc]
		) 
		send_goal_pick = sendgoal("pick")
		send_goal_kid = sendgoal("pick")
		kid_checker = kidnap_checker(send_goal_pick)
		canc_goal = cancel_goal(send_goal_pick.get_action_client())
		b41 = RSequence(name="Kidnap checking sequence", children=[kid_checker, canc_goal, b413, send_goal_kid]) # send goal still missing
		b42 = send_goal_pick 
		b40 = pt.composites.Selector(
			name="Nanvigation and kidnap checking",
			children=[b41, b42]
		)
		detect_cube_t1 = detectcube()
		pick_cube = movecube('pick')
		send_goal_place = sendgoal('place')
		place_cube = movecube('place')
		#detected_cube_t2 = cube_detected_on_table2(mode = 1)
		#detected_cube_t2 = improv_cube_detected_on_table2(mode = 1)

		b50 = RSequence(name="Task sequence", children=[b40, detect_cube_t1, pick_cube, send_goal_place, place_cube])
		
		b70 = resetpose(mode = 1)
		b71 = tuckarm()
		b72 = movehead("up")
		b73 = reset_tree_states(kid_checker, canc_goal, upd_loc, conv_checker, send_goal_pick, send_goal_kid,
                            detect_cube_t1, pick_cube, send_goal_place, place_cube, b72, b71, b70)
		b7 = RSequence(name="Repeat sequence", children=[b70, b71, b72, b73])

		b6 = pt.composites.Selector(
			name="Prepare for reset",
			children=[mission_done(mode = 1), b7]
		)

		b5 = RSequence(name="Run tries", children=[b50, b6])

		b4 =  pt.composites.Selector(
			name="First check and run",
			children=[mission_done(mode = 1), b5]
		) 
		
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b4])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()


# import py_trees as pt, py_trees_ros as ptr, rospy
# from behaviours_student import *
# from reactive_sequence import RSequence

# class BehaviourTree(ptr.trees.BehaviourTree):

# 	def __init__(self):

# 		rospy.loginfo("Initialising behaviour tree")

# 		b0 = tuckarm()
# 		b01 = movehead("up")
# 		blackboard = Blackboard()
# 		b1 = check_convergence(blackboard)
# 		b2 = update_localization(blackboard)
# 		b3 = move_pose("pick_pose", blackboard)
# 		b4 = detect_cube(blackboard)
# 		b5 = move_cube("pick", blackboard)
# 		b6 = move_pose("place_pose", blackboard)
# 		b7 = move_cube("place", blackboard)
# 		b9 = check_end(blackboard)

# 		tree = RSequence(name="Main sequence", children=[b0, b01, b1, b2, b3, b4, b5, b6, b7])
# 		super(BehaviourTree, self).__init__(tree)

# 		# execute the behaviour tree
# 		rospy.sleep(5)
# 		self.setup(timeout=10000)
# 		while not rospy.is_shutdown(): self.tick_tock(1)	

# if __name__ == "__main__":


# 	rospy.init_node('main_state_machine')
# 	try:
# 		BehaviourTree()
# 	except rospy.ROSInterruptException:
# 		pass

# 	rospy.spin()