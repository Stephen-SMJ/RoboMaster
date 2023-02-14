from detect import detect
import time
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import cv2
import datetime
import math
import threading
import eventlet
import sys

while(1):
    detect()
    
