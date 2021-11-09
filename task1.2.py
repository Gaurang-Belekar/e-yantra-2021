#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from task1_1 import *


class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('marker_detection') #Initialise rosnode 
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)
		
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        	# Subscribing to /camera/camera/image_raw

		self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker


# Callback function of amera topic
def image_callback(self, data):
# Note: Do not make this function lenghty, do all the processing outside this callback function
    try:
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
    except CvBridgeError as e:
        print(e)
        return
        
def maincode(self):
    self.cap= self.img
    while True:
        success, img =cap.read()
        arucoFound = task1_1.findArucoMarkers(img)
        if len(arucoFound[0])!=0:
                for bbox, id in zip(arucoFound[0], arucoFound[1]):
                    img = cv2.circle(img, (int(bbox[0][0][0]), int(bbox[0][0][1])), radius=4, color=(125, 125, 125), thickness=-1)
                    img = cv2.circle(img, (int(bbox[0][1][0]), int(bbox[0][1][1])), radius=4, color=(0, 255, 0), thickness=-1)
                    img = cv2.circle(img, (int(bbox[0][2][0]), int(bbox[0][2][1])), radius=4, color=(180,105,255), thickness=-1)
                    img = cv2.circle(img, (int(bbox[0][3][0]), int(bbox[0][3][1])), radius=4, color=(255,255,255), thickness=-1)
                    centrex=int((bbox[0][3][0]+bbox[0][1][0])/2)
                    centrey=int((bbox[0][3][1]+bbox[0][1][1])/2)
                    midx=int((bbox[0][0][0]+bbox[0][1][0])/2)
                    midy=int((bbox[0][0][1]+bbox[0][1][1])/2)
                    img = cv2.circle(img, (centrex,centrey), radius=4, color=(0,0,255), thickness=-1)
                    cv2.line(img,(centrex,centrey),(midx,midy),(255,0,0),4)
                    cv2.putText(img,str(id),(int((centrex+midx)/2),int((centrey+midy)/2)), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1,(0,0,255),2,cv2.LINE_AA)
                    if midx-centrex!=0:
                        slope=(midy-centrey)/(midx-centrex)
                        anglerad=math.atan(slope)
                        angledeg=math.degrees(anglerad)
                    else:
                        angledeg=90
                    
                    if midy>centrey:
                        if angledeg<0:
                            angledeg+=180
                    if midy<centrey:
                        if angledeg>0:
                            angledeg+=180
                        if angledeg<0:
                            angledeg+=360
                    if angledeg<=360 and angledeg>=180:
                        angledeg-=180
                    elif angledeg<=180 and angledeg>=0:
                        angledeg+=180            
                    
                    cv2.putText(img,str(int(angledeg)),(int(bbox[0][3][0]), int(bbox[0][3][1])), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1,(0,255,0),2,cv2.LINE_AA)
			           
			
def publish_data(self):
	self.marker_pub.publish(self.marker_msg)

if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
