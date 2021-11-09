import cv2
import numpy as np
import os
import math

def findArucoMarkers(img, markerSize=6, totalMarkers=250, draw=True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(cv2.aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = cv2.aruco.Dictionary_get(key)
    arucoParam = cv2.aruco.DetectorParameters_create()
    bboxs, ids, rejected = cv2.aruco.detectMarkers(imgGray,arucoDict, parameters=arucoParam)
    
    # if draw:
    #      cv2.aruco.drawDetectedMarkers(img, bboxs)
    return [bboxs, ids]


def augmentAruco(bbox, id, img, imgAug, drawId=True):
    tl = bbox[0][0][0], bbox[0][0][1]
    tr = bbox[0][1][0], bbox[0][1][1]
    br = bbox[0][2][0], bbox[0][2][1]
    bl = bbox[0][3][0], bbox[0][3][1]

    h,w,c=imgAug.shape
    pts1= np.array([tl,tr,br,bl])
    pts2=np.float32([[0,0],[w,0],[w,h],[0,h]])
    matrix,_=cv2.findHomography(pts2,pts1)
    imgOut=cv2.warpPerspective(imgAug, matrix, (img.shape[1],img.shape[0]))
    cv2.fillConvexPoly(img, pts1.astype(int),(0,0,0))
    imgOut=img+imgOut
    return imgOut

def maincode(image)
     cap = image
     imgAug=cv2.imread("red.png")

#augdics=loadAugImages("Markers")
toggle = 0
temp=0
while True:
    success, img =cap.read()
    arucoFound=findArucoMarkers(img)
    if len(arucoFound[0])!=0:
        for bbox, id in zip(arucoFound[0], arucoFound[1]):
            #img=augmentAruco(bbox, id, img, imgAug)
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
    cv2.imshow("Image",img)
    cv2.waitKey(1)
    if cv2.waitKey(1) == ord('q'):
        break
