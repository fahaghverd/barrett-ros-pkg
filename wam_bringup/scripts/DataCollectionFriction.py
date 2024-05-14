#!/usr/bin/python3

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from wam_msgs.msg import RTJointPos, RTJointVel
from wam_srvs.srv import JointMove
import json
import glob
from wam_srvs.srv import Hold
from numpy import linalg as LA

'''
from operator import truediv
import numpy as np
import rospy 

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from wam_msgs.msg import RTJointPos, RTJointVel
from wam_srvs.srv import JointMove
from wam_srvs.srv import Hold
from std_srvs.srv import Empty

import json
#import pickle
import os
import rosservice
#import pygame
#import keyboard
import glob
'''

'''
joint_state_data = []

# EE_pose_data = []

p = 0

key_pressed = []

# POS_READY = [
#     0.002227924477643431, 
#     -0.1490540623980915, 
#     -0.04214558734519736, 
#     1.6803055108189549, 
#     0.06452207850075688, 
#     -0.06341508205589094, 
#     0.01366506663019359,
# ]

POS_READY= [0.0009130838023128816, -1.9826090806117, 0.03375295956217106, 1.741920405799028]
#pygame.init()
#screen = pygame.display.set_mode((640, 480))
'''

class WAM(object):
    """abstract WAM arm definitions"""
    def __init__(self):
        self.pos = []
        self.vel = []
        self.joint_state_data = []
        self.joint_angle_bound = np.array([[-0.5, 0.5], [-2.5, -1.5], [-0.5, 0.5], [1.2, 2.2]]) # the reference angle is the home pose
        self.num_joints = 4
        self.joint = rospy.ServiceProxy('/leader/wam/hold_joint_pos', Hold) 
        self.collect = False
        self._init_joint_states_listener()

        # initialize publisher for jnt_pos_cmd and jnt_vel_cmd
        self.jnt_vel_pub = rospy.Publisher('/leader/wam/jnt_vel_cmd', RTJointVel, queue_size=1)
        self.jnt_pos_pub = rospy.Publisher('/leader/wam/jnt_pos_cmd', RTJointPos, queue_size=1)

        self.pos_home = self._read_home_pos()
        print("home pos received:", self.pos_home)
        

    def _init_joint_states_listener(self):
        """set up joint states listener from WAM control computer"""
        rospy.Subscriber('/leader/wam/joint_states', JointState, self._cb_joint_state)
        # rospy.spin() # i am not sure whether we should use it here or not

    def _cb_joint_state(self, data : JointState):
        self.pos = np.array(data.position)
        self.vel = np.array(data.velocity)
        joint_state = {'time': data.header.stamp.secs+data.header.stamp.nsecs*10^(-9),
                'position' : data.position,
                'velocity' : data.velocity,
                'effort' : data.effort}
        if self.collect:
            self.joint_state_data.append(joint_state)

    def _wait_for_joint_states(self):
        while len(self.pos) == 0:
            rospy.sleep(0.001)
    
    def _read_home_pos(self):
        self._wait_for_joint_states()
        return self.pos.copy()
    
    def joint_move(self, pos_goal: np.ndarray):
        """Move WAM to a desired position.
        q is a numpy array of length DoF that specifies the joint angles
        """
        # Communicate with /wam/joint_move service on control computer
        rospy.wait_for_service('/leader/wam/joint_move')
        try:
            joint_move_service = rospy.ServiceProxy('/leader/wam/joint_move', JointMove)
            joint_move_service(pos_goal)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    
    def joint_pos_cmd(self, pos_goal: np.ndarray):
                     
        msg = RTJointPos()
        msg.joints = pos_goal
        msg.rate_limits = np.array([500.0]*7)
        self.jnt_pos_pub.publish(msg)

    def joint_vel_cmd(self, vel_goal: np.ndarray):
                   
        msg = RTJointVel()
        msg.velocities = vel_goal
        self.jnt_vel_pub.publish(msg)
        print("velocity applied")

    def go_home(self):
        self._wait_for_joint_states()
        self.joint_move(self.pos_home)

    def stop(self):
        self.joint_vel_cmd([0.0, 0.0, 0.0, 0.0])

    def check_joint_bound(self):
        current_joint_angle = self.pos

        for i in range(self.num_joints):
            if current_joint_angle[i] > 0.05 + self.joint_angle_bound[i][1]:
                print("kesafat")
                print('Joint {} out of bound.'.format(i))
                return True
            elif current_joint_angle[i] < -0.05 + self.joint_angle_bound[i][0]:
                print('JOint {} out of bound.'.format(i))
                return True
            
        return False

class DataRecorder(object):
    def __init__(self, robot, joint_num):
        self.robot = robot
        self.joint_num = joint_num
        self.robot._wait_for_joint_states()
        self.steps = 10
        self.control_frequency = 50
        self.rate = rospy.Rate(self.control_frequency)

    def write_data(self, name = "vis"):
        data_id = len(glob.glob('/home/wam/data/friction_data/{}_*.json'.format(name)))
        out_file = open("/home/wam/data/friction_data/{}_{}.json".format(name, data_id), "w")
        json.dump(self.robot.joint_state_data, out_file)
        out_file.close()
        print('trajectory colleced and saved')
        self.robot.joint_state_data = []

    def collect_vis(self):

        
        count = 0
        rospy.sleep(3)
        self.robot.collect = True
        while True:
            velocity = [0.5, 0.0, 0.0, 0.0]
            self.robot.joint_vel_cmd(velocity)
            if robot.check_joint_bound() or count>self.steps:
                break
            count += 1
            self.rate.sleep()
        rospy.sleep(1)
        print('positive movement done.')
        self.robot.collect = False
        self.robot.stop()
        rospy.sleep(3)
        self.robot.go_home()
        rospy.sleep(4)
        self.robot.joint(False)

        self.robot.collect = True
        count = 0
        while True:
            velocity = [-0.5, 0.0, 0.0, 0.0]
            self.robot.joint_vel_cmd(velocity)
            if robot.check_joint_bound() or count>self.steps:
                break
            count += 1
            self.rate.sleep()
        rospy.sleep(1)
        print('negative movement done.')
        self.robot.collect = False
        self.robot.stop()
        rospy.sleep(3)
        self.robot.go_home()
        rospy.sleep(4)
        self.robot.joint(False)
        
        self.write_data("vis")

    def collect_col(self):
        
        joint_velocity = 0
        rospy.sleep(3)
        self.robot.collect = True
        while True:
            joint_velocity += 0.01
            velocity = [joint_velocity, 0.0, 0.0, 0.0]
            self.robot.joint_vel_cmd(velocity)
            velocity_error = np.abs(LA.norm(self.robot.vel) - joint_velocity)
            if robot.check_joint_bound() or LA.norm(self.robot.vel)>0.03:
                break
            self.rate.sleep()
        rospy.sleep(1)
        print('positive movement done.')
        self.robot.collect = False
        self.robot.stop()
        rospy.sleep(1)
        self.robot.go_home()
        rospy.sleep(2)
        self.robot.joint(False)

        self.robot.collect = True
        joint_velocity = 0
        while True:
            joint_velocity += 0.01
            velocity = [-joint_velocity, 0.0, 0.0, 0.0]
            self.robot.joint_vel_cmd(velocity)
            velocity_error = np.abs(LA.norm(self.robot.vel) - joint_velocity)
            if robot.check_joint_bound() or LA.norm(self.robot.vel)>0.03:
                break
            self.rate.sleep()
        rospy.sleep(1)
        print('negative movement done.')
        self.robot.collect = False
        self.robot.stop()
        rospy.sleep(1)
        self.robot.go_home()
        rospy.sleep(2)
        self.robot.joint(False)
        
        self.write_data("col")


if __name__ == '__main__':
    rospy.init_node("data_collection_node")
    
    robot = WAM()
    recorder = DataRecorder(robot, 4)
    recorder.collect_vis()
    recorder.collect_col() #Let's check the first one first!
    rospy.spin() # I am not sure whether we should use it here or not : Faezeh: you need it to make the callbacks running on highest frq as possible!

'''
def go_ready_pos():
        """Move WAM to a desired ready position.
        """
        joint_move(POS_READY)

def joint_move(pos_goal: np.ndarray):
        """Move WAM to a desired position.
        q is a numpy array of length 7 that specifies the joint angles
        """
  # Communicate with /wam/joint_move service on control computer
        rospy.wait_for_service('/leader/wam/joint_move')
        try:
            #print('found service')
            joint_move_service = rospy.ServiceProxy('/leader/wam/joint_move', JointMove)
            joint_move_service(pos_goal)
            #print('called move_q')
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)




def clip_velocity(vel, max_norm):
        vel_norm = np.linalg.norm(vel)
        if vel_norm > max_norm:
            print("clipped vel")
            return vel/vel_norm*max_norm # velocity rescaled to have max norm
        else:
            return vel

def joint_pos_cmd(pos_goal: np.ndarray, jnt_pos_pub):
                      
        msg = RTJointPos()
        # Publish to ROSfrom sensor_msgs.msg import JointState
        msg.joints = pos_goal
        msg.rate_limits = np.array([500.0]*7)
        jnt_pos_pub.publish(msg)
'''

'''
def init_joint_states_listener():
        """Set up joint_states listener from WAM control computer.
        """
        rospy.Subscriber('/wam/joint_states', JointState, callback_joint_state)
        
def init_EE_pose_listener():
        """Set up EE_pose listener from WAM control computer.
        """
        rospy.Subscriber('/wam/pose', PoseStamped, callback_EE_pose)

def init_key_pose_listener():
        """Set up keyobard listener from WAM control computer.
        """ 
        rospy.Subscriber('keyboard_command', String, callback_keyboard)
        
def callback_joint_state(data):
    #global joint_state_final
    #rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.position)
    #if self.pos == []: # first time we hear the joint states
    #    rospy.loginfo(rospy.get_caller_id() + ' VS system is reading joint states.')
    # print(data.header.stamp.secs)
    # print(list(data.position))
    # print(list(data.velocity))
    joint_state = {'time': data.header.stamp.secs+data.header.stamp.nsecs*10^(-9),
			   'position' : data.position,
			   'velocity' : data.velocity,
			   'effort' : data.effort}
    joint_state_data.append(joint_state)
    
def callback_EE_pose(data):
    #global EE_pose_final
    EE_pose = {'position' : (data.pose.position.x, data.pose.position.y, data.pose.position.z),
			   #'orientation' : data.pose.orientation}
			   'orientation': (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,  data.pose.orientation.w)}
    #EE_pose_final = EE_pose
    EE_pose_data.append(EE_pose)

def callback_keyboard(data):
    key_pressed = data
    trajs = len(glob.glob('/home/robot/DMP/DMP_data_joint*.json'))
    print("{} trajectories exist".format(trajs))

    if key_pressed == 'p':  # if key 'q' is pressed 
            print('Pick Trj Collected')
            out_file1 = open("/home/robot/DMP_data_joint_pick_{}.json".format(trajs), "w")
            json.dump(joint_state_data, out_file1)
            #print(joint_state_data)
            out_file1.close()
            out_file2 = open("/home/robot/DMP_data_EE_pick_{}.json".format(trajs), "w")
            json.dump(EE_pose_data, out_file2)
            out_file2.close()
            p = 1
            joint_state_data = []
            EE_pose_data = []
        
    if key_pressed == 'q' and p == 1:
            print('Place Trj Collected')
            #print(joint_state_data)
            out_file1.close()
            out_file2 = open("/home/robot/DMP/DMP_data_EE_place_{}.json".format(trajs), "w")
            json.dump(EE_pose_data, out_file2)
            out_file2.close()
'''

'''
Sends velocity commands ~200 Hz
Reads and stores pos, vel, torque (effort) at ~400 Hz
'''




'''
class DataRecorder(object):
    def __init__(self):
        rospy.Subscriber('/leader/wam/joint_states', JointState, self.cb_joint_state)
        rospy.Subscriber('keyboard_command', String, self.cb_keyboard)
        self.jnt_vel_pub = rospy.Publisher('/leader/wam/jnt_vel_cmd', RTJointVel, queue_size=10)
        self.jnt_pos_pub = rospy.Publisher('/leader/wam/jnt_pos_cmd', RTJointPos, queue_size=1)
        self.joint = rospy.ServiceProxy('/leader/wam/hold_joint_pos',Hold) 
        

        self.joint_state_data = []
        # self.ee_pose_data = []
        # self.picked = False
        self.collect = False

        # go_ready_pos()


    def join_vel_vis(self):
        Steps = 3
        rate = rospy.Rate(1) 

        i = 0
        while (not rospy.is_shutdown() or i<Steps):
            self.collect = True
            # Sleep to maintain the publishing rate
            print(i)
            msg = RTJointVel()
            msg.velocities = [0.4, 0, 0, 0]
            self.jnt_vel_pub.publish(msg)
            rate.sleep()
            i = i+1
        
        self.collect = False
        
        # how can I say to zero the velocity? also the constraint for position

        go_ready_pos()

        i = 0    
        while (not rospy.is_shutdown() or i<Steps):
            self.collect = True
            # Sleep to maintain the publishing rate
            print(i)
            msg = RTJointVel()
            msg.velocities = [-0.4, 0, 0, 0]
            self.jnt_vel_pub.publish(msg)
            rate.sleep()
            i = i+1
        self.collect = False

        data_id = len(glob.glob('/home/wam/data/friction_data/*.json'))
        print('Trj Collected')
        out_file1 = open("/home/wam/data/friction_data/vis{}.json".format(data_id), "w")
        json.dump(self.joint_state_data, out_file1)
        out_file1.close()
        self.joint_state_data = []

    def join_vel_col(self):
        # Steps = 3
        rate = rospy.Rate(1) 

        i = 0
        while (not rospy.is_shutdown() or i<Steps):
            self.collect = True
            # Sleep to maintain the publishing rate
            print(i)
            msg = RTJointVel()
            msg.velocities = [0.4, 0, 0, 0]
            self.jnt_vel_pub.publish(msg)
            rate.sleep()
            i = i+1
        
        self.collect = False
        
        # how can I say to zero the velocity?

        go_ready_pos()

        i = 0    
        while (not rospy.is_shutdown() or i<Steps):
            self.collect = True
            # Sleep to maintain the publishing rate
            print(i)
            msg = RTJointVel()
            msg.velocities = [-0.4, 0, 0, 0]
            self.jnt_vel_pub.publish(msg)
            rate.sleep()
            i = i+1
        self.collect = False

        data_id = len(glob.glob('/home/wam/data/friction_data/*.json'))
        print('Trj Collected')
        out_file1 = open("/home/wam/data/friction_data/col{}.json".format(data_id), "w")
        json.dump(self.joint_state_data, out_file1)
        out_file1.close()
        self.joint_state_data = []
    
    def cb_joint_state(self, data : JointState):
        joint_state = {'time': data.header.stamp.secs+data.header.stamp.nsecs*10^(-9),
                'position' : data.position,
                'velocity' : data.velocity,
                'effort' : data.effort}
        if self.collect:
            self.joint_state_data.append(joint_state)


    def cb_keyboard(self, key_pressed : String):
        data_id = len(glob.glob('/home/wam/data/friction_data/*.json'))
        if key_pressed.data == 's':
            print('Data collecting started')
            self.collect = True
                
        if key_pressed.data == 'e':
            print('Trj Collected')
            out_file1 = open("/home/wam/data/friction_data/{}.json".format(data_id), "w")
            json.dump(self.joint_state_data, out_file1)
            out_file1.close()
            self.joint_state_data = []
            self.collect = False
                
        if key_pressed.data == 'g':
            go_ready_pos()
            self.joint(False) #To be able to move the arm
        


if __name__ == '__main__':
    rospy.init_node("DMP_node")
    # Create a ROS publisher


    # init_joint_states_listener()
    # init_EE_pose_listener()
    # init_key_pose_listener()    
    # go_ready_pos()
	
    # joint = rospy.ServiceProxy('/wam/hold_joint_pos',Hold) 
    # joint(False) #To be able to move the arm

    recorder = DataRecorder()
    

    recorder.join_vel_col()
    rospy.spin()
    """
    while not rospy.is_shutdown():
        #joint_state_data.append(joint_state_final)
        #EE_pose_data.append(EE_pose_final)	
        print (i)
        i = i+1
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    print('Pick Trj Collected')
                    out_file1 = open("/home/robot/DMP/DMP_data_joint_pick_{}.json".format(trajs), "w")
                    json.dump(joint_stat            chmod +xe_data, out_file1)
						#print(joint_state_data)
                    out_file1.close()
                    out_file2 = open("/home/robot/DMP/DMP_data_EE_pick_{}.json".format(trajs), "w")
                    json.dump(EE_pose_data, out_file2)
                    out_file2.ckey_pressedlose()
                    p = 1

                if event.key == pygame.K_RIGHT and p == 1:
                    json.dump(joint_state_data, out_file1)
					#print(joint_state_data)
                    out_file1.close()
                    out_file2 = open("/home/robot/DMP/DMP_data_EE_place_{}.json".format(trajs), "w")
                    json.dump(Ekey_pressedE_pose_data, out_file2)
                    out_file2.close()

                    
        if key_pressed == 'p':  # if key 'q' is pressed 
            print('Pick Trj Collected')
            out_file1 = open("/home/robot/DMP_data_joint_pick_{}.json".format(trajs), "w")
            json.dump(joint_state_data, out_file1)
            #print(joint_state_key_pressedata, out_file2)
            out_file2.close()
            p = 1/zeus/bhand/initialize
            
        #rate.sleep()   
    """	  
'''
    
    
