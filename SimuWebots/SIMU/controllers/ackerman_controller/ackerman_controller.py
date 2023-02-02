"""ackerman_controller controller."""

import rospy 
import numpy as np 
import message_filters  

from vehicle import Driver
from controller import Lidar, Camera, DistanceSensor
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray

class RosController : 
    
    def __init__(self, driver, max_speed=2.0, max_angle=0.35) :
        
        # Robot initialisation 
        self.driver  = driver 
        self.lidar   = self.driver.getDevice("RPlidar A2")
        self.camera  = self.driver.getDevice("camera")
        s_left  = self.driver.getDevice("ds_left")
        s_right = self.driver.getDevice("ds_right")
        s_back_left  = self.driver.getDevice("ds_back_left")
        s_back_right = self.driver.getDevice("ds_back_right")

        self.sensors = [s_left, s_right, s_back_left, s_back_right]
        self.time_step = 64
        self.camera.enable(self.time_step)
        for s in self.sensors : s.enable(self.time_step)
        
        # Movement parameters 
        self.max_speed, self.max_angle = max_speed, max_angle
        self.current_speed, self.current_angle = 0.0, 0.0
        
        # Sensor publishers 
        self.lidar_scan, self.image_scan, self.sensors_scan = Float32MultiArray(), Int16MultiArray(), Float32MultiArray() 
        self.pub_scan = rospy.Publisher("/LidarScan", Float32MultiArray, queue_size=1)
        self.pub_img  = rospy.Publisher("/ImageScan", Int16MultiArray, queue_size=1)
        self.pub_sens = rospy.Publisher("/SensorsScan", Float32MultiArray, queue_size=1)
   

    def apply_command(self, msg_speed, msg_angle) : 
        """ Applying speed to vehicle motors """
         
        self.current_speed = msg_speed.data * self.max_speed
        self.current_angle = msg_angle.data * self.max_angle 
        
        # Clipping to max value
        if self.current_speed > self.max_speed : self.current_speed = self.max_speed     
        elif self.current_speed < -1*self.max_speed : self.current_speed = -1*self.max_speed        
        if self.current_angle > self.max_angle : self.current_angle = self.max_angle
        elif self.current_angle < -1*self.max_angle : self.current_angle = -1*self.max_angle 

        # Applying speed 
        driver.setCruisingSpeed(self.current_speed)
        driver.setSteeringAngle(self.current_angle)
   

    def publish_scan(self, event) :
        try : 
            self.lidar.enable(self.time_step)
            self.lidar_scan.data = self.lidar.getRangeImage() 
            self.pub_scan.publish(self.lidar_scan)
            self.lidar.disable()
        except : print("[WRN] - Lidar encountered an issue at this iteration") ; return 
        
  
    def publish_image(self, event) : 
        self.image_scan.data = self.camera.getImage()  
        self.pub_img.publish(self.image_scan)
      
    def publish_sensors(self, event) : 
        self.sensors_scan.data = [s.getValue() for s in self.sensors]
        self.pub_sens.publish(self.sensors_scan)
   
   
print("ROS Controller initialisation") 
driver = Driver()      
rospy.init_node('ackerman_controller', anonymous=True)

TIME_SCAN = 2.0/10.0
TIME_IMG  = 5.0/10.0
TIME_SENS = 1.0/10.0

rc = RosController(driver) 
rospy.Timer(rospy.Duration(TIME_SCAN), rc.publish_scan)
rospy.Timer(rospy.Duration(TIME_IMG),  rc.publish_image)
rospy.Timer(rospy.Duration(TIME_SENS), rc.publish_sensors)
print("Controller initialized : publishing in ROS topics : \n-/LidarScan : Lidar datas\n-/ImageScan : Camera datas\n-/SensorsScan : TOFs datas")

while driver.step() != -1: 
     
    # Reading speed/angle commands 
    topic_speed = rospy.get_param("speed_topic", default="/SpeedCommand")
    topic_angle = rospy.get_param("angle_topic", default="/AngleCommand")
    speed_sub = message_filters.Subscriber(topic_speed, Float32, queue_size=1, buff_size=1024)
    angle_sub = message_filters.Subscriber(topic_angle, Float32, queue_size=1, buff_size=1024)
    
    # Applying commands
    ts = message_filters.ApproximateTimeSynchronizer([speed_sub, angle_sub], queue_size=1, slop=0.1, allow_headerless=True)
    ts.registerCallback(rc.apply_command)
  
