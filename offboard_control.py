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
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

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
        self.local_pos = Point(0.0, 0.0, 10.0)
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
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    setpoints = [] #List to setpoints
    sp1 = PoseStamped()
    sp2 = PoseStamped()
    sp3 = PoseStamped()
    sp4 = PoseStamped()
    sp5 = PoseStamped()
    sp1.pose.position.x = 0
    sp1.pose.position.y = 0
    sp1.pose.position.z = 10
    sp2.pose.position.x = 10
    sp2.pose.position.y = 0
    sp2.pose.position.z = 10
    sp3.pose.position.x = 10
    sp3.pose.position.y = 10
    sp3.pose.position.z = 10
    sp4.pose.position.x = 0
    sp4.pose.position.y = 10
    sp4.pose.position.z = 10
    sp5.pose.position.x = 0
    sp5.pose.position.y = 0
    sp5.pose.position.z = 0
    setpoints.append(sp1)
    setpoints.append(sp2)
    setpoints.append(sp3)
    setpoints.append(sp4)
    setpoints.append(sp1)
    setpoints.append(sp5)


    # Similarly initialize other publishers 

    # Create empty message containers 
    pos =PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 5.0
    vel.linear.y = 5.0
    vel.linear.z = 5.0
    
    # Similarly add other containers 

    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

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
    while  stateMt.state.mode != "OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")
    i = 0
    pos = setpoints[i]
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


        pos = setpoints[i]
        stateMt.sp = setpoints[i]

        # print("check3")
        while True:
            rospy.Subscriber('mavros/local_position/pose', PoseStamped, stateMt.posCb)
            r1 = pos.pose.position.x - stateMt.local_pos.x
            r2 = pos.pose.position.y - stateMt.local_pos.y
            r3 = pos.pose.position.z - stateMt.local_pos.z
            r = math.sqrt(r1 * r1 + r2 * r2 + r3 * r3)

            stateMt.sp = setpoints[i]
            local_pos_pub.publish(pos)
            # for j in range(30):
            #     local_pos_pub.publish(pos)
            #     rate.sleep()
            #local_vel_pub.publish(vel)
            if r <= 2 and i<5:
                i+=1
                # print("check")
                pos = setpoints[i]
                stateMt.sp = setpoints[i]
                rate.sleep()
                for j in range(100):
                    local_pos_pub.publish(pos)
                    rate.sleep()
                break;
            elif i<5:
                # print("21")
                i+=1
                pos = setpoints[i]
                stateMt.sp = setpoints[i]
                for j in range(100):
                    local_pos_pub.publish(pos)
                    rate.sleep()
        local_pos_pub.publish(pos)
        # print("check2")
        if i==5:
            ofb_ctl.setAutoLandMode()
            rate.sleep()
            for j in range(300):
                # local_pos_pub.publish(pos)
                rate.sleep()
            ofb_ctl.setDisarm()

        if i==5:
            break;
        #local_vel_pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass