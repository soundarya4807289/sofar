#!/usr/bin/env python  
import rospy
import sys 
import tf2_ros
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from human_baxter_collaboration.msg import *
from human_baxter_collaboration.srv import *
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState
from collections import Counter
import math


MiddlePlacementN_ = Pose()
red_CubesList_ = [0,0,0,0,0,0]
blue_CubesList_ = [0,0,0,0,0] 
count1_ = 0
count2_ = 0
Blue_box_ = [0,0,0,0,0]
human_left_arm_joint_states_ = []
human_right_arm_joint_states_ = []
robot_right_arm_joint_states_ = []
robot_left_arm_joint_states_ = []
robot_right_arm_idle_state_ = []
robot_left_arm_idle_state_ = []


def clbk_baxter_jointstates(msg):
    global count2_
    global robot_right_arm_joint_states_
    global robot_left_arm_joint_states_
    #print("In clbk baxter joinstates")
        
    global robot_right_arm_idle_state_
    global robot_left_arm_idle_state_

    #print(count2_)
    count2_ = 1
    
    robot_right_arm_joint_states_ = msg.position[1:8]
    robot_left_arm_joint_states_ = msg.position[8:15]


def clbk_unitytf(msg):
    global count1_
    global MiddlePlacementN_
    global Blue_box_
    global human_left_arm_joint_states_
    global human_right_arm_joint_states_
    global blue_CubesList_
    global red_CubesList_
    
    #print("In clbk unitytf function")
    #print(count1_)
    count1_ = 1
    MiddlePlacementN_ = msg.frames[23].pose
    #M
    Blue_boxL_ = msg.frames[24].pose
    Blue_box_[0] = Pose()
    Blue_box_[0].position.x = 0.6721628904342651
    Blue_box_[0].position.y = 0.5736517310142517
    Blue_box_[0].position.z = 0.8265659213066101 + 0.05
    Blue_box_[0].orientation.x = 0.010405738838016987
    Blue_box_[0].orientation.y = -0.999945878982544
    Blue_box_[0].orientation.z = 4.548492971157003e-10
    Blue_box_[0].orientation.w = 4.370902217942785e-08
    #E 
    Blue_box_[1] = Pose()
    Blue_box_[1].position.x = 0.6112496256828308
    Blue_box_[1].position.y = 0.6069132685661316
    Blue_box_[1].position.z = 0.8260000348091125 + 0.05
    Blue_box_[1].orientation.x = 0.0012575702276080847
    Blue_box_[1].orientation.y = 0.9999992251396179
    Blue_box_[1].orientation.z = 5.497013955135799e-11
    Blue_box_[1].orientation.w = -4.371135275960114e-08
    #C
    Blue_box_[2] = Pose()
    Blue_box_[2].position.x = 0.7384998798370361
    Blue_box_[2].position.y = 0.6425093412399292
    Blue_box_[2].position.z = 0.8259978294372559 + 0.05
    Blue_box_[2].orientation.x = 0.0007537920610047877
    Blue_box_[2].orientation.y = 0.9999997019767761
    Blue_box_[2].orientation.z = 3.294929606934005e-11
    Blue_box_[2].orientation.w = -4.3711374075883214e-08
    #I
    Blue_box_[3] = Pose()
    Blue_box_[3].position.x = 0.6732463240623474
    Blue_box_[3].position.y = 0.6416134834289551
    Blue_box_[3].position.z = 0.8265742063522339 + 0.05
    Blue_box_[3].orientation.x = 0.0005569640779867768
    Blue_box_[3].orientation.y = -0.9999998211860657
    Blue_box_[3].orientation.z = 2.4345673046988203e-11
    Blue_box_[3].orientation.w = 4.371138118131057e-08
    #G
    Blue_box_[4] = Pose()
    Blue_box_[4].position.x = 0.7399289011955261
    Blue_box_[4].position.y = 0.5729789733886719
    Blue_box_[4].position.z = 0.8260290026664734 + 0.05
    Blue_box_[4].orientation.x = 0.0027120306622236967
    Blue_box_[4].orientation.y = -0.9999963045120239
    Blue_box_[4].orientation.z = 1.1854663084509554e-10
    Blue_box_[4].orientation.w = 4.371122841462238e-08
    
    
    #waypoints_ = msg.frames[24].pose
    
    human_left_arm_joint_states_ = msg.frames[4:7]
    human_right_arm_joint_states_ = msg.frames[7:10]
    
    blue_CubesList_[0] = msg.frames[20].pose   # M
    blue_CubesList_[1] = msg.frames[11].pose   # E
    blue_CubesList_[2] = msg.frames[13].pose   # C
    blue_CubesList_[3] = msg.frames[16].pose   # I
    blue_CubesList_[4] = msg.frames[17].pose   # G
   
    red_CubesList_[0] = msg.frames[10].pose    # A
    red_CubesList_[1] = msg.frames[12].pose    # L
    red_CubesList_[2] = msg.frames[14].pose    # B
    red_CubesList_[3] = msg.frames[15].pose    # H
    red_CubesList_[4] = msg.frames[18].pose    # D
    red_CubesList_[5] = msg.frames[19].pose    # F
    #count1_ = count1_ + 1	    
	    
def isRobotMoving():
	robot_right_arm_idle_state_.append(robot_right_arm_joint_states_)
	robot_left_arm_idle_state_.append(robot_left_arm_joint_states_)
	#print(robot_right_arm_idle_state_)
	#print(robot_left_arm_idle_state_)
	rospy.sleep(5.)
	robot_right_arm_idle_state_.append(robot_right_arm_joint_states_)
	robot_left_arm_idle_state_.append(robot_left_arm_joint_states_)
	#print(robot_right_arm_idle_state_)
	#print(robot_left_arm_idle_state_)

	if(set(robot_right_arm_idle_state_[0])==set(robot_right_arm_idle_state_[1]) and set(robot_left_arm_idle_state_[0])==set(robot_left_arm_idle_state_[1])):
		robot_right_arm_idle_state_.clear()
		robot_left_arm_idle_state_.clear()
		return False
	else:
		robot_right_arm_idle_state_.clear()
		robot_left_arm_idle_state_.clear()
		return True
		

def main():
	global pub_BaxterTrajectory
	rospy.init_node('sofar_project')
	sub_baxter_jointstates = rospy.Subscriber('/baxter_joint_states', JointState, clbk_baxter_jointstates)
	pub_BaxterTrajectory = rospy.Publisher('/baxter_moveit_trajectory', BaxterTrajectory, queue_size=1)
	sub_unitytf = rospy.Subscriber('/unity_tf', UnityTf, clbk_unitytf)
	client_left = rospy.ServiceProxy('baxter_moveit_left_arm_trajectory', TrajectoryService)
	client_right = rospy.ServiceProxy('baxter_moveit_right_arm_trajectory', TrajectoryService)
		
	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		req = TrajectoryServiceRequest()
		if(count1_ == 1 and count2_ == 1):
			
			# This condition checks if Human is approaching to pick up the first red cubes.
			if(human_left_arm_joint_states_[2].pose.position.x <= 0.95):												      
				print("Human Robot Collaboration Simulation Started! :")
				
				#This loop will iterate for all five blue cubes.
				for i in range(5):
				
					# Condition until blue_CubesList_[i] is not in the bluebox this loop will not sto	
					while not ( (0.5769 <= blue_CubesList_[i].position.x <= 0.7577) and (0.5573 <= blue_CubesList_[i].position. y <= 0.6540)):
						print("Check no 1")
						
						#This is condition checkes if the robot arm is moving or not. 
						if(isRobotMoving()== False):
							print("Check no 2")
							
							# This condition checks if the current desired blue cube is in the blue box or not. 
							if (blue_CubesList_[i].position.y + 0.05 <= MiddlePlacementN_.position.y):
								print("Check no 3.right")
								
								# This condition checks if the service is available to compute the robot right arm trajectory.
								rospy.wait_for_service('baxter_moveit_right_arm_trajectory')				
								try:
									arm = "right"
									req.joints_input.joint_00 = robot_right_arm_joint_states_[0]
									req.joints_input.joint_01 = robot_right_arm_joint_states_[1]
									req.joints_input.joint_02 = robot_right_arm_joint_states_[2]
									req.joints_input.joint_03 = robot_right_arm_joint_states_[3]
									req.joints_input.joint_04 = robot_right_arm_joint_states_[4]
									req.joints_input.joint_05 = robot_right_arm_joint_states_[5]
									req.joints_input.joint_06 = robot_right_arm_joint_states_[6]
									req.pick_pose = blue_CubesList_[i]
									req.pick_pose.position.z = req.pick_pose.position.z + 0.05
									req.place_pose = MiddlePlacementN_
									req.place_pose.position.z = req.place_pose.position.z + 0.05  
									#print(req)
									response = client_right(req)
									print("Sending request..")
									#wait for 1 sec
									rospy.sleep(2.)
									print(response)
									pub_BaxterTrajectory.publish(arm, response.trajectories)						    
								except rospy.ServiceException as e:
									print("Service call failed: %s"%e)
							else:
								print("Check no 3.left")
								
								# This condition checks if the service is available to compute the robot left arm trajectory.
								rospy.wait_for_service('baxter_moveit_left_arm_trajectory')
								try:
									arm = "left"
									req.joints_input.joint_00 = robot_left_arm_joint_states_[0]
									req.joints_input.joint_01 = robot_left_arm_joint_states_[1]
									req.joints_input.joint_02 = robot_left_arm_joint_states_[2]
									req.joints_input.joint_03 = robot_left_arm_joint_states_[3]
									req.joints_input.joint_04 = robot_left_arm_joint_states_[4]
									req.joints_input.joint_05 = robot_left_arm_joint_states_[5]
									req.joints_input.joint_06 = robot_left_arm_joint_states_[6]
									req.pick_pose = blue_CubesList_[i]
									req.pick_pose.position.z = req.pick_pose.position.z + 0.05
									req.place_pose = Blue_box_[i]
									#req.place_pose.position.z = req.place_pose.position.z + 0.05
									#print(req)
									response = client_left(req)
									rospy.sleep(2.)
									print("Sending request..")
									print(response)
									pub_BaxterTrajectory.publish(arm, response.trajectories)
									 #wait for 1 sec
								except rospy.ServiceException as e:
									print("Service call failed: %s"%e)										
				print("The simulation is over yayy!!!!")
				break
		r.sleep()
			
if __name__ == '__main__':
    main()
