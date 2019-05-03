#!/usr/bin/env python
import roslib
import sys
import numpy as np
import rospy
from wheely_camera.msg import Wheely_contour_info, Detected_objs_info
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

CONTOUR_FILTER_LIMIT = 0.02
SMALLEST_AREA_TO_DETECT = 400
QUEUE_SIZE = 10
class TrackBar:

    def __init__(self):
        self.img=np.zeros((300, 512, 3), np.uint8)
        cv2.namedWindow('trackbar_image')
        cv2.createTrackbar('R', 'trackbar_image', 0, 255, self.nothing)
        cv2.createTrackbar('G', 'trackbar_image', 0, 255, self.nothing)
        cv2.createTrackbar('B', 'trackbar_image', 0, 255, self.nothing)
    def getBGR(self):
        r = cv2.getTrackbarPos('R', 'trackbar_image')
        g = cv2.getTrackbarPos('G','trackbar_image')
        b = cv2.getTrackbarPos('B','trackbar_image')  
        return (b, g, r)
    def nothing(self):
        pass
        
  
class Image_reciver:
    def __init__(self):
        self.bg = CvBridge()
        self.image = rospy.Subscriber("/my_camera/camera1/image_raw", Image, self.callback)
        self.pub = rospy.Publisher('wheely/image/detected_obj', Detected_objs_info, queue_size=QUEUE_SIZE)
        self.detectedObjs = []
        # this track bar is used for testing purposes, 
        self.trackbar = TrackBar()
    def callback(self, data):
        try:

            img = (self.bg.imgmsg_to_cv2(data, desired_encoding="bgr8"))
            # the color to detect is recvied from color track bar for testing!!! Not production MODE
            cv_image, mask = self.mask(img, self.trackbar.getBGR())
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            contours = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)[1]
            self.detectedObjs = []
            for cnt in contours:
                moments = cv2.moments(cnt) 
                try:
                    objInfo = Wheely_contour_info()
                    objInfo.x, objInfo.y = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                    area = cv2.contourArea(cnt)
                    approx = cv2.approxPolyDP(cnt, CONTOUR_FILTER_LIMIT * cv2.arcLength(cnt, True), True) 
                    shape = 0
                    if area > SMALLEST_AREA_TO_DETECT:
                        cv2.drawContours(img, [approx], 0, (0, 0, 0), 2)
                        shape = len(approx)
                    objInfo.shape = shape
                    self.detectedObjs.append(objInfo)
                except ZeroDivisionError:
                    pass
            try:
                self.publishData()
            except:
                pass
    
            print(self.detectedObjs)
        except CvBridgeError as e:
            print(e)
        timg = self.trackbar.img
        cv2.imshow("trackbar_image", timg)
        timg[:]= list(self.trackbar.getBGR())
        cv2.imshow("Image window", img)
        cv2.waitKey(1)
    def cropImage(self, img, descenter = 0, rows_to_watch = 120):
        h, w, ch = img.shape
        return img[(h)/2+descenter:(h)/2+(descenter+rows_to_watch)][1:w]
    def mask(self, img, color=(0,255,252), upper_range=15, lower_range=15):
        bgr_color = np.uint8([[[color[0],color[1], color[2]]]])
        hsv_color = cv2.cvtColor(bgr_color, cv2.COLOR_BGR2HSV)
        h = hsv_color[0][0][0]
        lower_range = np.array([h - 15, 50, 50])
        upper_range = np.array([h + 15,255,255])
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #lower_range = np.array([0, 100, 100])
        #upper_range = np.array([50, 255, 255])
        mask = cv2.inRange(hsv, lower_range, upper_range)
        res = cv2.bitwise_and(img, img, mask=mask)
        return res, mask
    def getCentroid(self, mask, h, w):
        moments = cv2.moments(mask, False)
        try:
            cx, cy = moments['m10']/moments['m00'], moments['m01']/moments['m00']
        except ZeroDivisionError:
            cx, cy = h/2, w/2
        return cx, cy
    def publishData(self):
        msg = Detected_objs_info()
        msg.data = self.detectedObjs
        self.pub.publish(msg)
def main():
    image_reciver = Image_reciver()
    rospy.init_node("Wheely_img_processor", anonymous=True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()