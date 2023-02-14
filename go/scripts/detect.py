

#!/usr/bin/env python3
import time
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge
import numpy as np
import cv2
import datetime
import math
import threading
import eventlet
import sys
import timeout_decorator

target_low, target_up = np.array([5, 230, 100]), np.array([20, 255, 255]) #上下限

class IMAGE_LEFT:
    def __init__(self,image=None,x=None,y=None,xmax=None,ymax=None):
        print("init")
        self.sub = rospy.Subscriber('/airsim_node/drone_1/front_left/Scene', Image, self.image_callback,queue_size=2)
        #self.sub = rospy.Subscriber('/airsim_node/drone_1/front_left/Scene/camera_info', String, self.String_callback)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback) # practical velocity

        self.image=image
        self.x = x
        self.y = y
        self.xmax = xmax
        self.ymax = ymax
    def image_callback(self, msg): # feedback means actual value.
        br = CvBridge()
        left_im = br.imgmsg_to_cv2(msg) # Convert the message to a new image
        self.image = left_im
        hsv=cv2.cvtColor(self.image,cv2.COLOR_BGR2HSV) #转hsv
        #去除颜色范围外的其余颜色
        mask = cv2.inRange(hsv, target_low, target_up)
        #cv2.imshow('mask',mask)
        # 二值化操作
        ret, binary = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5), np.uint8)
        dilation = cv2.dilate(binary, kernel, iterations=1) #膨胀
        #cv2.imshow('pengzhang',dilation)
        contours, hierarchy = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #找轮廓
        if len(contours) > 0:
            boxes = [cv2.boundingRect(c) for c in contours]
            largebox, boxsize = [], []
            for x, y, w, h in boxes:
                if w*h >= 500:
                    largebox.append((x,y,w,h))
                    boxsize.append(w*h)
            x,y,w,h = largebox[boxsize.index(max(boxsize))]
            xmin,xmax,ymin,ymax = x, x+w, y, y+h 
            self.x = xmin
            self.y = ymin
            self.xmax = xmax
            self.ymax = ymax

    def odom_callback(self, msg): 
        print("---ODOM_callback---")
        self.angle_z = msg.twist.twist.angular.z
        self.linear_x = msg.twist.twist.linear.x
    def String_callback(self, msg): 
        print(msg)
        print("---String_callback---")
   
        
        
class IMAGE_RIGHT:
    def __init__(self,image=None,x=None,y=None,xmax=None,ymax=None):
        print("init")
        self.sub = rospy.Subscriber('/airsim_node/drone_1/front_right/Scene', Image, self.image_callback,queue_size=2)
        #self.sub = rospy.wait_for_message('/airsim_node/drone_1/front_right/Scene', Image, self.image_callback, timeout=None)
        self.image=image
        self.x = x
        self.y = y
        self.xmax = xmax
        self.ymax = ymax

    def image_callback(self, msg): # feedback means actual value.
        #print("---right_IMAGE_callback---")
        br = CvBridge()
        right_im = br.imgmsg_to_cv2(msg) # Convert the message to a new image
        self.image=right_im
        hsv=cv2.cvtColor(self.image,cv2.COLOR_BGR2HSV) #转hsv
        #去除颜色范围外的其余颜色
        mask = cv2.inRange(hsv, target_low, target_up)
        # 二值化操作
        ret, binary = cv2.threshold(mask, 0, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5), np.uint8)
        dilation = cv2.dilate(binary, kernel, iterations=1) #膨胀
        contours, hierarchy = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #找轮廓
        if len(contours) > 0:
            boxes = [cv2.boundingRect(c) for c in contours]
            largebox, boxsize = [], []
            for x, y, w, h in boxes:
                if w*h >= 500:
                    largebox.append((x,y,w,h))
                    boxsize.append(w*h)
            x,y,w,h = largebox[boxsize.index(max(boxsize))]
            xmin,xmax,ymin,ymax = x, x+w, y, y+h
            self.x = xmin
            self.y = ymin
            self.xmax = xmax
            self.ymax = ymax
        image_process(a, b, 320, 95, 0.1179)
        
def rad_caculate(a,b):
    c=np.sqrt(a*a+b*b)
    l=a+b+c
    s=a*b/2
    A=math.asin(a/c)
    B=math.asin(b/c)
    return [A,B]
    
#positions=[min_x,min_y,max_x,max_y]
def image_process(image_left,image_right,f,B,pexl):
    pose_pub = rospy.Publisher('/target', Point32, queue_size=10)

    left_image = image_left.image
    right_image = image_right.image
    left_positions = [image_left.x,image_left.y,image_left.xmax,image_left.ymax]
    right_positions = [image_right.x,image_right.y,image_right.xmax,image_right.ymax]
    image_x = 640
    image_y = 480
    
    left_positions_int=left_positions  #转成int
    right_positions_int=right_positions #转成int
    left_position_x=(left_positions_int[0]+left_positions_int[2])/2 #计算左目相机x上中心点
    right_position_x=(right_positions_int[0]+right_positions_int[2])/2   #计算右目相机图x上中心点
    temp=left_position_x-right_position_x    #两边差距（公式1）
    distance=int(f)*int(B/pexl)/(temp)    #测距
    sd=left_position_x-image_x/2     #左视图上x点与左目相机中心点x距离
    left_rad=rad_caculate(sd,f)[0]   #左视图,无人机飞行y方向偏角
    left_image_center_to_point_x=math.tan(left_rad)*distance   #以左视图中心点为原点 无人机飞行y坐标
    offset=left_image_center_to_point_x - B/pexl/2  #矫正到无人机中心点
    x_rad=rad_caculate(offset,distance)[0]   #无人机中心点与实际点偏移角度
    
    y_p=math.tan(x_rad)*distance*0.00011785   #无人机中心点为原点，无人机飞行y坐标
    
    left_position_y=(left_positions_int[1]+left_positions_int[3])/2    #计算左目相机y上中心点  
    right_position_y=(right_positions_int[1]+right_positions_int[3])/2    #计算右目相机图y上中心点
    
    y_rad = math.atan(((left_position_y+right_position_y)/2-image_y/2)/f)   #计算无人机在水平方向向下偏移角度
    z_p = math.tan(y_rad)*distance*0.00011785    #无人机中心点为原点，无人机飞行z坐标
    x_p = distance*0.00011785
    print(x_p,y_p,z_p)
    target = Point32()
    target.x = x_p
    target.y = y_p
    target.z = z_p
    #print(type(z_p),type(y_p),type(distance),)
    pose_pub.publish(target)
    return [x_p,y_p,z_p]

def thread_job():
    rospy.spin()

if __name__ == '__main__':

    rospy.init_node('IMAGE', anonymous=False)
    a = IMAGE_LEFT()
    b = IMAGE_RIGHT()
    
    try:
        #detect()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            print("IMAGE Node open!")
            
    except KeyboardInterrupt:
        # self.motor_stop()
        print("Exiting Program")
    except Exception as exception_error:
        print("Error occurred. exiting program")
        print("Error : "+str(exception_error))

    finally:
        # control_speed_fun(speed=0)
        # self.serial_port.close()
        
        print("OVER IMAGE SUB CONTORL")

    pass

