#!/usr/bin/env python
from __future__ import print_function

import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import constants as Constant
import time

class ObjectByColorDetector:

  def __init__(self):
    self.image_loc_pub = rospy.Publisher("/wheely/detected_object_location", Point, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/wheely/camera/image_raw",Image,self.callback)

  def callback(self,data):
    global rate
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #defining the range of Yellow color
    # yellow_lower = np.array([22,60,200],np.uint8)
    # yellow_upper = np.array([60,255,255],np.uint8)

    red_lower = np.array([0,120,70],np.uint8)
    red_upper = np.array([10,255,255],np.uint8)

    #finding the range yellow colour in the image
    red = cv2.inRange(hsv, red_lower, red_upper)
    
    #Tracking Colour (Red) 
    (_,contours,hierarchy)=cv2.findContours(red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    img = cv_image
    (h, w) = img.shape[:2]

    # M = cv2.getRotationMatrix2D((w/2, h/2), -90, 1.0)
    # img = cv2.warpAffine(img, M, (h, w))   
    # img = cv2.flip(img,1)

    x=y= 0
    maxArea = 0

    contour = self.getBigImg(contours)
    # for pic, contour in enumerate(contours):
    #     xt,yt,wt,ht = cv2.boundingRect(contour)
    #     if wt*ht > maxArea:
    #       maxArea = wt*ht
    x,y,w,h = cv2.boundingRect(contour)
    print("X - ", Constant.CAMERA_HEIGHT-y-(w/2), " Y - ", Constant.CAMERA_WIDTH-x-(h/2), " W- ", w, " H- ", h)         
    img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),3)

    M = cv2.getRotationMatrix2D((Constant.CAMERA_WIDTH/2, Constant.CAMERA_HEIGHT/2), -90, 1.0)
    img = cv2.warpAffine(img, M, (Constant.CAMERA_HEIGHT, Constant.CAMERA_WIDTH))   
    cv2.imshow("Color Tracking",img)
                            
    if cv2.waitKey(10) & 0xFF == 27:
        cap.release()
        cv2.destroyAllWindows()

    try:
      self.image_loc_pub.publish(Point(Constant.CAMERA_HEIGHT-y-(w/2), Constant.CAMERA_WIDTH-x-(h/2), 0))
      rate.sleep()

    except CvBridgeError as e:
      print(e)

  def getBigImg(self, contours):
    bigCon = None
    maxArea = 0
    for pic, contour in enumerate(contours):
        xt,yt,wt,ht = cv2.boundingRect(contour)
        if wt*ht > maxArea:
          maxArea = wt*ht
          bigCon = contour

    return bigCon

rate = None

def main():
  global rate
  ic = ObjectByColorDetector()
  rospy.init_node('ColoredObjectDetector')
  rate = rospy.Rate(Constant.OBJ_DET_RATE)

  print("Colored object detection node initiated")
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()