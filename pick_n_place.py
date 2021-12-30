#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode.
See the documentation for offboard mode in px4 here() to understand more about offboard mode
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose


'''

import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class offboard_control:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming',
                                            mavros_msgs.srv.CommandBool)  # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s" % e)

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Autoland Mode could not be set." % e)

    # Similarly delacre other service proxies

    def gripper_True(self):
        rospy.wait_for_service('/activate_gripper')
        try:
            gripper_service = rospy.ServiceProxy('/activate_gripper', mavros_msgs.srv.CommandBool)
            gripper_service('True')
        except rospy.ServiceException as e:
            print("service activate_gripper call failed: %s.Gripper Could not be activated" % e)

    def gripper_False(self):
        rospy.wait_for_service('/activate_gripper')
        try:
            gripper_service = rospy.ServiceProxy('/activate_gripper', mavros_msgs.srv.CommandBool)
            gripper_service('False')
        except rospy.ServiceException as e:
            print("service activate_gripper call failed: %s.Gripper Could not be activated" % e)

    def offboard_set_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:

            ModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)

            ModeService(custom_mode="OFFBOARD")

        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Offboard Mode could not be set." % e)

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure


class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.local_pos = Point(0.0, 0.0, 3.0)
        self.sp = PositionTarget()

    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers


def main():
    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    rate = rospy.Rate(20.0)

    # Make the list of setpoints
    setpoints = []  # List to setpoints
    sp1 = PositionTarget()
    sp2 = PositionTarget()
    sp3 = PositionTarget()
    sp4 = PositionTarget()
    sp5 = PositionTarget()
    sp6 = PositionTarget()
    sp1.position.x = 0
    sp1.position.y = 0
    sp1.position.z = 3

    sp2.position.x = 3
    sp2.position.y = 0
    sp2.position.z = 3
    sp3.position.x = 3
    sp3.position.y = 0
    sp3.position.z = -1
    sp4.position.x = 3
    sp4.position.y = 3
    sp4.position.z = 3
    sp5.position.x = 3
    sp5.position.y = 3
    sp5.position.z = -1
    sp6.position.x = 0
    sp6.position.y = 0
    sp6.position.z = -1
    setpoints.append(sp1)
    setpoints.append(sp2)
    setpoints.append(sp3)
    setpoints.append(sp4)
    setpoints.append(sp5)
    setpoints.append(sp4)
    setpoints.append(sp1)
    setpoints.append(sp6)

    # Similarly initialize other publishers

    # Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0


    # Similarly add other containers

    # Initialize subscriber
    rospy.Subscriber("/mavros/state", State, stateMt.stateCb)

    # Similarly initialize other subscribers

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    for i in range(100):
        local_pos_pub.publish(pos)
        # stateMt.sp = setpoints[0]
        rate.sleep()

    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    while stateMt.state.mode != "OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print("OFFBOARD mode activated")
    i = 0
    pos.pose.position = stateMt.local_pos
    stateMt.sp = setpoints[i]
    for j in range(150):
        local_pos_pub.publish(pos)
        rate.sleep()

    # Publish the setpoints
    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here 
        '''

        pos.pose.position = setpoints[i].position
        stateMt.sp = setpoints[i]

        print("check3")
        while True:
            rospy.Subscriber('mavros/local_position/pose', PositionTarget, stateMt.posCb)
            pos.pose.position = setpoints[i].position
            r1 = stateMt.sp.position.x - stateMt.local_pos.x
            r2 = stateMt.sp.position.y - stateMt.local_pos.y
            r3 = stateMt.sp.position.z - stateMt.local_pos.z
            r = math.sqrt(r1 * r1 + r2 * r2 + r3 * r3)

            stateMt.sp = setpoints[i]
            local_pos_pub.publish(pos)
            # for j in range(30):
            #     local_pos_pub.publish(pos)
            #     rate.sleep()
            # local_vel_pub.publish(vel)
            if r <= 0.5 and i < 6:

                i += 1
                if i == 3:
                    grip_val = rospy.Subscriber('/gripper_check', String)
                    while grip_val != 'True':
                        ofb_ctl.gripper_True()
                        pos.pose.position = setpoints[i].position
                        stateMt.sp = setpoints[i]
                        rate.sleep()


                elif i == 5:
                    grip_val = rospy.Subscriber('/gripper_check', String)
                    while grip_val != 'False':
                        ofb_ctl.gripper_False()
                        pos.pose.position = setpoints[i].position
                        stateMt.sp = setpoints[i]
                        rate.sleep()
                print("check")
                pos.pose.position = setpoints[i].position
                stateMt.sp = setpoints[i]
                rate.sleep()
                for j in range(100):
                    local_pos_pub.publish(pos)
                    rate.sleep()
                break;
            elif i < 6:
                # print("21")

                i += 1
                if i == 3:
                    grip_val = rospy.Subscriber('/gripper_check', String)
                    while grip_val != 'True':
                        ofb_ctl.gripper_True()
                        pos.pose.position = setpoints[i].position
                        stateMt.sp = setpoints[i]
                        rate.sleep()

                elif i == 5:
                    grip_val = rospy.Subscriber('/gripper_check', String)
                    while grip_val != 'False':
                        ofb_ctl.gripper_False()
                        pos.pose.position = setpoints[i].position
                        stateMt.sp = setpoints[i]
                        rate.sleep()
                pos.pose.position = setpoints[i].position
                stateMt.sp = setpoints[i]
                for j in range(100):
                    local_pos_pub.publish(pos)
                    rate.sleep()
            else:
                break;
        local_pos_pub.publish(pos)
        print("check2")
        if i == 6:
            ofb_ctl.setAutoLandMode()
            rate.sleep()
            for j in range(300):
                # local_pos_pub.publish(pos)
                rate.sleep()
            ofb_ctl.setDisarm()

        if i == 6:
            break;
        # local_vel_pub.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass