#!/usr/bin/env python3

from tkinter import N
from numpy import flip
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from white_location.cfg import WhiteLocConfig
import numpy as np

light_cond_pub = rospy.Publisher('light_cond', Bool, queue_size=10)
bridge = CvBridge()

def dyn_rcfg_cb(config, level):
  global thresh, flip
  thresh = config.threshold
  flip = config.flip
  return config

def image_callback(ros_image):
  global bridge
  try: #convert ros_image into an opencv-compatible image
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
    print(e)
  #from now on, you can work exactly like with opencv
  if flip == True:
    cv_image = cv2.flip(cv_image, 1) # flip to see ourselves in a mirror
  cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
  (rows, cols, channels) = cv_image.shape

  gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

  ret, bw_image = cv2.threshold(gray_image, thresh, # input image, threshold_value,
                                255, cv2.THRESH_BINARY) # max value in image, threshold type
                                # ret: threshold value used
  #num_white_pix = np.sum(bw_image == 255) alternative way to count # of white pixels
  seg_size = int(cols / 3)
  num_seg_pix = rows * seg_size
  pct_whitePixel_L = 100*cv2.countNonZero(bw_image[0:rows, 0:seg_size])/num_seg_pix
  pct_whitePixel_C = 100*cv2.countNonZero(bw_image[0:rows, seg_size:2*seg_size])/num_seg_pix
  pct_whitePixel_R = 100*cv2.countNonZero(bw_image[0:rows, 2*seg_size:])/num_seg_pix   # may need to correct for the last seg.
  print(f"L = {pct_whitePixel_L:.1f}%, C = {pct_whitePixel_C:.1f}%, R = {pct_whitePixel_R:.1f}%,")
  if   ((pct_whitePixel_L > pct_whitePixel_C) and (pct_whitePixel_L > pct_whitePixel_R)):
    print("Left")
  elif ((pct_whitePixel_R > pct_whitePixel_C) and (pct_whitePixel_R > pct_whitePixel_L)):
    print("Right")
  else:
    print("Center")
  cv2.imshow("Image window", bw_image)
  cv2.waitKey(3)
  
if __name__ == '__main__':
  rospy.init_node('white_loc', anonymous=True)
  imgtopic = rospy.get_param("~imgtopic_name") # private name
  rospy.Subscriber(imgtopic, Image, image_callback)
  srv = Server(WhiteLocConfig, dyn_rcfg_cb)
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass

  # CJ Oct 14, 2022