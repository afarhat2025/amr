#!/usr/bin/env python3
import json
import logging
import math
import os
import random
import re
import time
import cv2
import subprocess
import datetime
import traceback
from dataclasses import dataclass
from pymongo import MongoClient, ReturnDocument
from typing import List
import pdb
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError

from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSHistoryPolicy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Polygon, Point32
from nav_msgs.msg import OccupancyGrid
import rclpy.qos
import rclpy.qos_event
import rclpy.time
from sensor_msgs.msg import Image, LaserScan, Joy
from std_msgs.msg import Bool,String
from rtabmap_msgs.msg import Info
from amr_v4_msgs_srvs.msg import Robot, Pin, Mode, Error, Lights
from sick_scan_xd.srv import FieldSetWriteSrv,FieldSetReadSrv
from sick_scan_xd.msg import LIDoutputstateMsg

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from phoenix6 import controls, configs, hardware, signals, StatusCode, CANBus
from amr_v4_msgs_srvs.msg import Charging,Battery

def get_db(db: str):
    """MongoDB"""
    try:
        client = MongoClient("mongodb://mrumel:56.583fudge@10.10.1.45:27017/?authSource=admin")
        my_db = client[db]
        return my_db
    except Exception as err:
        logging.error("Error connecting to MongoDB: %s", err)

@dataclass
class AMR:
    amr: int
    errors: List[str]
    totalcycles: int
    batterystatus: float
    stop: bool
    station_curr: str
    mission_type: str
    generated_path: str
    amr: int
    current_pose: str

@dataclass
class Inter_Porj_3:
    id: int
    ins: List[int]
    out: List[int]
    occupied: bool
    priority: bool

@dataclass
class Inter_Porj_6:
    id: int
    rfid_in: List[int]
    inter_point_in: str
    inter_point_out: str
    occupied: bool
    stop_dist: float

class SW_ESTOP(Node):
    
    def __init__(self):
        super().__init__("estop_nav",namespace=robot_name)
        self.mode_publisher_mode_ = self.create_publisher(Mode, "/"+robot_name+"/mode",10)
        self.mode_subscriber_joy = self.create_subscription(Joy, "/"+robot_name+"/joy", self.mode_switch, 10)
        self.estop_subscriber = self.create_subscription(Bool, "/"+robot_name+"/estop", self.estop_callback, 10)
        #self.lidar_output_subscriber = self.create_subscription(LIDoutputstateMsg, "/"+robot_name+"/sick/lidoutputstate", self.lidar_output_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)
        self.robot_mode = 'Manual' # 0 is manual, 1 is Auto
        self.mode_subscriber_joy
        #self.lidar_output_subscriber
        self.not_clear = False
        self.lf_oper = False
        self.pin_down = False
        self.global_estop = False
    
    def mode_switch(self,msg):
        mode_ = Mode()
        if self.robot_mode == 'Auto':
            mode_.mode = True
            mode_.inter_stop = self.not_clear
            mode_.line_follow = self.lf_oper

        elif self.robot_mode =='Manual':
            mode_.mode = False
            mode_.inter_stop = False
            mode_.line_follow = False
        
        
        self.mode_publisher_mode_.publish(mode_)

        if msg.buttons[3]:
            if not mode_.mode:
                self.robot_mode = 'Auto'
        elif msg.buttons[1]:
            if mode_.mode:
                self.robot_mode = 'Manual'
        elif msg.buttons[4]: #pindown
            self.pin_down = True
        elif msg.buttons[5]: #pinup
            self.pin_down = False

    
    def estop_callback(self,msg):
        if msg.data:
            self.get_logger().info("E-Stop is ON")
            self.global_estop = True
        else:
            self.global_estop = False

    
    
class LocalizationCheck(Node):
    """
    Checks if we are localized in home to continue to Navigation
    """
    def __init__(self):
        super().__init__("localize_info",namespace=robot_name)
        self.recieved_message = False
        self.publisher_ = self.create_publisher(Robot, "/"+robot_name+"/diagnostic", 10)
        self.subscriber_ = self.create_subscription(Info, "/"+robot_name+"/info", self.callback_localize, 10)    
        self.subscriber_localize = self.create_subscription(PoseWithCovarianceStamped, "/"+robot_name+"/localization_pose",self.get_localization_pose,10)
        self.subscriber_
        self.subscriber_localize
        self.localization = False
        self.current_robot_mission_status = "Robot is Currently Booting and Initializing"
        self.initial_pose = PoseWithCovarianceStamped()
        self.tag_id = 0
        self.global_info = []


    def callback_localize(self,msg):
        message = Robot()

        if (msg.header.frame_id != None and msg.loop_closure_id != None): # msg.landmark_id != 0): # change id as needed for Home at begin (TODO)
            self.recieved_message = True
            #if (msg.landmark_id !=0):
                #self.get_logger().info('I saw the tag : "%i"' % msg.landmark_id)

        if (self.recieved_message):
            message.robot_localization_status = 'The Robot is Localized and Starting to Navigate !'
            message.localized = True
            message.robot_active_status = self.current_robot_mission_status
            self.localization = True
        else:
            message.robot_localization_status = 'Please drive the Robot to the Tag at HOME !'
            message.localized = False
            self.localization = False
            self.current_robot_mission_status = "Please drive Robot to Home Station in front of tag to Start Navigating"
        self.publisher_.publish(message)

    
    def get_localization_pose(self,msg):
        if (self.recieved_message):
            self.initial_pose = msg
        else:
            self.get_logger().info('The Current robot position is not available yet')

class PinPublisher(Node):
    """class for pin commands """
    def __init__(self,sw_estop_instance):
        super().__init__('Pin_Status',namespace=robot_name)
        self.publisher = self.create_publisher(Pin, "/"+robot_name+"/pin_cmd", 10)
        self.timer = 0.25
        self.sw_estop = sw_estop_instance
        self.timer = self.create_timer(self.timer, self.callback)
        self.pinValue = False
        self.initial_reset = True
        self.error = False
        self.pin_upper_sf = -1.34
        self.pin_upper_limit = -1.38
        self.pin_lower_sf = 0.00
        self.talonfx = hardware.TalonFX(3,"can0")
        self.timeout = 10.0
        self.current_limit = 9.0
        self.safe_reset_current_lim = 32.0
        self.max_stator_curr_reset = 40.0
    
    def callback(self):
        message = Pin()
        pin_down = self.sw_estop.pin_down
        robot_mode = self.sw_estop.robot_mode
        #print(f"the value of pincmd Auto: {self.pinValue} and reset is : {self.initial_reset} and pin value for manual is: {pin_down}")
        if self.initial_reset:
            message.pin_command = int(self.pinValue)
            self.talonfx.clear_sticky_faults()
            self.configure_talonfx_soft_limits(limits=False)
            self.initial_reset_routine()
        if not self.initial_reset and robot_mode == 'Manual':
            if pin_down: # pinup
                self.talonfx.set_control(controls.MotionMagicDutyCycle(position=self.pin_upper_sf))
            elif not pin_down:
                self.talonfx.set_control(controls.MotionMagicDutyCycle(position=self.pin_lower_sf))
        if not self.initial_reset and robot_mode == 'Auto':
            if (self.pinValue): # pinup
                message.pin_command = int(self.pinValue)
                self.talonfx.set_control(controls.MotionMagicDutyCycle(position=self.pin_upper_sf))

            elif (self.pinValue is False):
                message.pin_command = int(self.pinValue)
                self.talonfx.set_control(controls.MotionMagicDutyCycle(position=self.pin_lower_sf))

            self.publisher.publish(message)

    def initial_reset_routine(self):
        
        start_time = time.time()
        
        while self.talonfx.get_torque_current().value < self.max_stator_curr_reset:
            current_time = time.time() - start_time
            time.sleep(1.0)
            self.talonfx.set_control(controls.VoltageOut(output=0.7))
            #print(f"stator current {str(self.talonfx.get_torque_current())} and rotor position is {str(self.talonfx.get_position())} ")
            if  self.talonfx.get_stator_current().value >= self.safe_reset_current_lim or current_time > self.timeout:
                self.talonfx.set_control(controls.VoltageOut(output=0.0))
                break
        
        self.configure_talonfx_soft_limits(limits=False)
        #self.talonfx.set_control(controls.MotionMagicDutyCycle(position=self.pin_upper_sf))
        time.sleep(1.0)
        self.initial_reset = False

    def configure_talonfx_soft_limits(self,limits):
        self.talonfx.clear_sticky_faults()
        self.talonfx.get_position().set_update_frequency(50)
        cfg = configs.TalonFXConfiguration()
        
        self.talonfx.configurator.set_position(0.00)
        #print(f"Current position is: {self.talonfx.get_position()}")
        self.talonfx.configurator.refresh(cfg)
        cfg.software_limit_switch.forward_soft_limit_enable = limits
        cfg.software_limit_switch.reverse_soft_limit_enable = True
        cfg.software_limit_switch.forward_soft_limit_threshold = self.pin_lower_sf
        cfg.software_limit_switch.reverse_soft_limit_threshold = self.pin_upper_limit
        cfg.current_limits.supply_current_limit = self.current_limit
        cfg.current_limits.supply_current_limit_enable = True

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0,5,1):
            status = self.talonfx.configurator.apply(cfg)
            if status.is_ok():
                break
            elif not status.is_ok():
                self.get_logger(f"Could not apply configs, error code: {status.name}, going to kill program")
                break

class ServerConnect(Node):
    def __init__(self):
        super().__init__('server',namespace=robot_name) 
        self.subscriber = self.create_subscription(String,'/server_mi_4_0/mach2api',self.server_callback,1)
        self.subscriber
        self.forklift_cmd = False
        self.unloader_cmd = False

    def server_callback(self,msg):
        msg_list = msg.data.split(';',-1)
        self.forklift_cmd = False
        self.unloader_cmd = False
        if msg_list[0] == 'FORK' and msg_list[1] == 'OK':
            self.forklift_cmd = True
        elif msg_list[0] == 'Unloaded' and msg_list[1] == 'Run':
            self.unloader_cmd = True
        elif msg_list[0] == 'Unloading' and msg_list[1] == 'WIP':
            self.unloader_cmd = False
        self.get_logger().info(f"Received: {msg.data}")

class ChargerCMD(Node):
    def __init__(self):
        super().__init__('battery',namespace=robot_name) 
        self.publisher_ = self.create_publisher(Charging, '/'+robot_name+'/charger_cmd', 1)
        self.subscriber = self.create_subscription(Battery,'/'+robot_name+'/battery_state',self.battery_feedback,10)
        self.publisher_lights = self.create_publisher(Lights, '/'+robot_name+'/lights', 1)
        self.timer = self.create_timer(1.0, self.publish_charging_command)
        self.robot_charge_cmd = False
        #self.estop_sw = estop_sw
        self.c = 0.0
        self.battery_temp = ''
        self.battery_current = 0.0
        self.battery_weak_switch = ''
        self.battery_soc = 0.0
        self.unloader = False
        self.forklift = False
        self.home = False
        self.subscriber

    def publish_charging_command(self):
        msg = Charging()
        msg_1 = Lights()
        if self.robot_charge_cmd: #and self.estop_sw.robot_mode == 'Auto':
            msg.start_charging_main = True
            msg_1.charging = True
        elif not self.robot_charge_cmd:# and self.estop_sw.robot_mode == 'Auto':
            msg.start_charging_main = False
            msg_1.charging = False
        if self.unloader:
            msg_1.unload = True
        elif not self.unloader:
            time.sleep(1.0)
            msg_1.unload = False
        if self.forklift:
            msg_1.forklift = True
        elif not self.forklift:
            time.sleep(1.0)
            msg_1.forklift = False
        if self.home:
            msg_1.home = True
        elif not self.home:
            time.sleep(1.0)
            msg_1.home = False
        self.publisher_.publish(msg)
        self.publisher_lights.publish(msg_1)
        #self.get_logger().info(f'Publishing: {msg.start_charging_main}')
    
    def battery_feedback(self,msg):
        #print(f"Battery soc: {msg.soc} and battery current is {msg.current}")
        self.battery_current = float(msg.current)
        self.battery_temp = msg.temp
        self.battery_soc = float(msg.soc)
        self.battery_weak_switch = msg.weak_switch

class ChangeFootprint(Node):
    def __init__(self):
        super().__init__('Change_footprint',namespace=robot_name)
        self.publisher_local = self.create_publisher(Polygon, "/"+robot_name+'/local_costmap/footprint', 10)
        self.publisher_global = self.create_publisher(Polygon, "/"+robot_name+'/global_costmap/footprint', 10)
        self.timer = self.create_timer(1.0, self.callback)
        self.decrease_footprint = True

    def callback(self):
        new_footprint = Polygon()

        if self.decrease_footprint:
            #footprint with cart
            new_points = [
                Point32(x=0.55, y=0.31, z=0.00), #A
                Point32(x=0.55, y=-0.31, z=0.00), #B
                Point32(x=-0.55, y=-0.31, z=0.00), #C
                Point32(x=-0.55, y=-0.60, z=0.00), #D
                Point32(x=-1.19, y=-0.60, z=0.00), #E
                Point32(x=-1.19, y=0.60, z=0.00), #F
                Point32(x=-0.55, y=0.60, z=0.00), #G
                Point32(x=-0.55, y=0.31, z=0.00)] #H
        else:
            #footprint without cart,
            new_points = [
                Point32(x=0.55, y=0.31, z=0.00),
                Point32(x=0.55, y=-0.31, z=0.00),
                Point32(x=-0.55, y=-0.31, z=0.00),
                Point32(x=-0.55, y=0.31, z=0.00)]
            
        new_footprint.points = new_points
        self.publisher_local.publish(new_footprint)
        self.publisher_global.publish(new_footprint)
        #self.get_logger().info('Changed footprint published: %s' % new_footprint)

class Linefollower(Node):
    """Line Follower class for docking to charger"""

    def __init__(self):
        super().__init__('Line_Tracker',namespace=robot_name)
        self.bridge = CvBridge()
        self.MIN_AREA = 1000
        self.MIN_AREA_TRACK = 800
        self.LINEAR_SPEED = 0.06
        self.KP = 0.0009
        self.LOSS_FACTOR = 1.2
        self.TIMER_PERIOD = 0.07
        self.FINALIZATION_PERIOD = 2
        self.MAX_ERROR = 225
        self.width_offset = -93
        self.lower_green = np.array([40,  17,  0])
        self.upper_green = np.array([85, 70, 235])
        self.lower_red = np.array([0, 32, 50]) 
        self.upper_red = np.array([179, 255, 119])
        self.lower_red_2 = np.array([0, 32, 50])
        self.upper_red_2 = np.array([15, 255, 255])
        self.publisher = self.create_publisher(Twist, "/"+robot_name+'/cmd_vel_tracker',10)
        self.subscription = self.create_subscription(Image, '/'+robot_name+'/zed_node/rgb/image_rect_color',
                                             self.image_callback,
                                             QoSProfile(depth=10))
        self.subscription
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)
        self.error = 0
        self.mark_side = "right"
        self.average_pixels = 0
        self.green_pixels = 0
        self.ext_control = False
        self.angular_z = 0
        self.linear_x = 0
        self.extension_control = False
        self.running = ""
        self.Track_Mark_Min_Area = 100000000
        self.min_area = 2000
        self.latch_count = False
        self.searching_for_line = False
        self.search_step = 0
        self.step_counter = 0
        self.cycle_count = 0
        self.tmp_cycle = 0
        self.first_time = False
        self.green_detected_frames = 0
        self.green_lost_frames =0 
        self.prev_gray = None
        self.shape = None
        self.first_time_angular = False
        self.thres =70
        self.initial_mission = "Station"
        self.station_release_charger = False
        self.counter = 0
        self.image_input = None
        self.image_input_hsv = None

        self.just_seen_line = False
        self.just_seen_right_mark = False
        self.should_move = False
        self.right_mark_count = 0
        self.finalization_countdown =  None
        self.request_to_check  = False
        self.robot_tunnel_vision = False

    def initial_reset(self):
        self.error = 0
        self.mark_side = "right"
        self.average_pixels = 0
        self.green_pixels = 0
        self.ext_control = False
        self.angular_z = 0
        self.linear_x = 0
        self.extension_control = False
        self.running = ""
        self.Track_Mark_Min_Area = 100000000
        self.min_area = 2000
        self.latch_count = False
        self.searching_for_line = False
        self.search_step = 0
        self.step_counter = 0
        self.cycle_count = 0
        self.tmp_cycle = 0
        self.first_time = False
        self.green_detected_frames = 0
        self.green_lost_frames = 0
        self.prev_gray = None
        self.shape = None
        self.first_time_angular = False
        self.thres = 70
        self.initial_mission = "Station"
        self.station_release_charger = False
        self.counter = 0
        self.image_input = None
        self.image_input_hsv = None
        self.just_seen_line = False
        self.just_seen_right_mark = False
        self.should_move = False
        self.right_mark_count = 0
        self.finalization_countdown = None
        self.request_to_check = False
        self.robot_tunnel_vision = False


    def start_stop_follower(self,command):

        if command and self.first_time:
            print("Line detected! Starting line following.")
            self.should_move = True
            #self.searching_for_line = True
        # elif command and self.first_time:
        #     print("Line not detected! Initiating search.")
        #     should_move = False
        #     finalization_countdown = None
        #     if self.tmp_cycle == 0:
        #         self.searching_for_line = True
        #         self.tmp_cycle += 1
        #     self.search_step = 0
        #     self.step_counter = 0
        elif not command:
            self.should_move = False
            self.finalization_countdown = None
            self.searching_for_line = False
            self.first_time = False
            self.tmp_cycle = 0
            self.stop_robot()

    def image_callback(self,data):
    
        try:
            self.image_input = self.bridge.imgmsg_to_cv2(data,'bgr8') #bgr 
            self.image_input_hsv = cv2.cvtColor(self.image_input, cv2.COLOR_BGR2HSV) #hsv
        except CvBridgeError as e:
            self.get_logger().info(e)
            
    def timer_callback(self):
        """
        Main Driving Function
        """
        global crop_w_start
        
        if type(self.image_input) != np.ndarray:
            self.get_logger().info("No image input")
            return
        else:
            self.first_time = True

        height, width, _ = self.image_input.shape
        image_hsv = self.image_input_hsv.copy()
        if self.initial_mission == "Station":
            self.thres = 30
        else:
            self.thres = 70

        crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = self.crop_size(height, width)
        if self.robot_tunnel_vision:
            crop_w_start = crop_w_start + 400
            crop = image_hsv[(crop_h_start):(crop_h_stop),(crop_w_start):(crop_w_stop-230)]
        else:
            crop_w_start = crop_w_start
            crop = image_hsv[crop_h_start:crop_h_stop,crop_w_start:crop_w_stop]
        crop_red = image_hsv[(crop_h_start+150):crop_h_stop-25,(crop_w_start+125):(crop_w_stop)]
        mask = cv2.inRange(crop, self.lower_green, self.upper_green)
        mask_red_1 = cv2.inRange(crop_red, self.lower_red, self.upper_red)
        mask_red_2 = cv2.inRange(crop_red,self.lower_red_2, self.upper_red_2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        #mask_red = cv2.inRange(crop_red, self.lower_red, self.upper_red)
        h, s, v = cv2.split(crop_red)
        _, sat_mask = cv2.threshold(s, self.thres, 255, cv2.THRESH_BINARY)
        _, val_mask = cv2.threshold(v, 90, 255, cv2.THRESH_BINARY)
        combined_mask = cv2.bitwise_and(mask_red, cv2.bitwise_and(sat_mask, val_mask))
        mask_red_filtered = cv2.bitwise_and(mask_red, sat_mask)
        mask_red_filtered = cv2.bitwise_and(mask_red_filtered, val_mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
        noise_green = cv2.morphologyEx(mask, cv2.MORPH_RECT, kernel, iterations=1)
        noise_red = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        #noise_red = cv2.morphologyEx(noise_red, cv2.MORPH_CLOSE, kernel, iterations=1)
        self.average_pixels = cv2.countNonZero(noise_red)
        self.green_pixels = cv2.countNonZero(noise_green)
        #self.shape = self.get_shape_data(noise_red, mask_red)
        #print(f"Shape is: {self.shape}"_)
        while self.average_pixels > 9200 and self.counter == 0 and self.initial_mission == "Station" and self.first_time_angular:
            self.stop_robot()
            if self.station_release_charger:
                self.counter += 1
                break
    
        line, self.mark_side, area = self.get_contour_data(mask,noise_green, image_hsv[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])
        message = Twist()
        if not self.first_time_angular:
            max_z_exception = 2.50
        else :
            max_z_exception = 0.22
        if line:
            x = line['x']
            self.error = x - width//2 + self.width_offset
            message.linear.x = self.LINEAR_SPEED
            self.just_seen_line = True
            # if self.error < -250:  
            #     self.error = max(self.error, 1)  
            #     message.angular.z = float(self.error) * -self.KP
            # elif self.error > 250:  
            #     self.error = min(self.error,-1)  
            #     message.angular.z = float(self.error) * -self.KP
            
        else:
            if self.just_seen_line:
                self.just_seen_line = False
                #error = error * self.LOSS_FACTOR
                self.error = 0
            message.linear.x = 0.0
        
        
        if self.green_pixels > 40000 or abs(message.angular.z) > max_z_exception:
            print("Green pixels too high or angular velocity too high")
            self.green_detected_frames = 0
            self.request_to_check = True
            self.latch_count = False
            

        if (self.green_pixels > 12000 and not self.latch_count and abs(self.error) < 250) and self.request_to_check or self.initial_mission == "Unloader":
            self.green_lost_frames = 0
            print("In green detection condition")
            self.green_detected_frames += 1
            self.lower_green = np.array([40, 17, 0])
            self.upper_green = np.array([85, 70, 235])
            if self.green_detected_frames > 5:
                self.green_detected_frames = 0
                self.latch_count = True
                self.request_to_check = False

        if self.green_pixels <= 8000 and abs(self.error) < 250:
            self.green_lost_frames += 1
            
            print("In dynamic HSV switch condition")
            self.lower_green = np.array([30, 0, 0])
            self.upper_green = np.array([130, 255, 200])
            if abs(message.angular.z) > 0.14 or abs(self.error) > 240:
                self.latch_count = False
                self.green_detected_frames = 0
            
                
        # if (self.green_pixels <= 4500 or abs(self.error) > 300 or abs(message.angular.z) > 0.20 or 
        #     (self.mark_side is None and self.latch_count)):
                
        #         self.green_lost_frames += 1
        #         if self.green_lost_frames > 5:
        #             self.first_time_angular = True
        #             self.latch_count = False
        #             #self.initial_search = False
        #             request_to_check = True
        #             print("Latch reset due to out-of-zone condition")


        if abs(-self.KP * self.error) >= max_z_exception and self.initial_mission == "Station":
            message.angular.z = 0.0
            #message.linear.x = 0.0
        elif abs(-self.KP * self.error) < max_z_exception and self.initial_mission == "Station":
            message.angular.z = -self.KP * self.error
        elif self.initial_mission == "Unloader":
            message.angular.z = -self.KP * self.error
        self.angular_z =  message.angular.z

        if self.mark_side != None:
            if (self.mark_side == "right") and (self.finalization_countdown == None) and \
                (abs(self.error) <= self.MAX_ERROR) and (not self.just_seen_right_mark):
                self.right_mark_count += 1
                if self.right_mark_count == 1 and self.extension_control == True:
                    self.finalization_countdown = int(self.FINALIZATION_PERIOD / self.TIMER_PERIOD) + 1  
                self.just_seen_right_mark = True
        else:
            self.just_seen_right_mark = False
        
        #print("Error: {} | Mark side: {} | x: {} | z: {}, ".format(self.error, self.mark_side,message.linear.x, message.angular.z))
        #print(f"Green pixels: {self.green_pixels} | Area: {area} | red: {self.average_pixels}")
        if self.finalization_countdown != None:
            if self.finalization_countdown > 0:
                self.finalization_countdown -= 1

            elif (self.finalization_countdown == 0 or self.finalization_countdown < 0) and self.average_pixels > 20000:
                #time.sleep(1)
                #should_move = False
                pass
        
        if self.searching_for_line:
            line, self.mark_side, area = self.get_contour_data(mask,noise_green, image_hsv[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])
            if area > 3000:
                print("Line found! Stopping search and starting line following.")
                self.searching_for_line = False
                self.stop_robot()
                self.should_move = True
                return

            twist = Twist()
            if self.search_step == 0:
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                time.sleep(6.0)
                self.step_counter += 1
                if area > 3500:
                    self.stop_robot()
                    return
                if self.step_counter >= 4:
                    self.search_step = 1
                    self.step_counter = 0

            elif self.search_step == 1:
                twist.linear.x = 0.0
                twist.angular.z = -0.1
                self.publisher.publish(twist)
                time.sleep(1.0)
                self.step_counter += 1
                if area > 3500:
                    self.stop_robot()
                    return
                if self.step_counter >= 4:
                    self.search_step = 2  # Switch to left twirl
                    self.step_counter = 0
            
            # Step 3: Twirl center, checking for green
            elif self.search_step == 2:
                twist.linear.x = 0.0
                twist.angular.z = 0.1
                self.publisher.publish(twist)
                time.sleep(1.0)
                self.step_counter += 1
                if area > 3500:
                    self.stop_robot()
                    return
                if self.step_counter >= 4:
                    self.search_step = 3 
                    self.step_counter = 0

            elif self.search_step == 3:
                twist.linear.x = 0.0
                twist.angular.z = 0.1
                self.publisher.publish(twist)
                time.sleep(0.5)
                self.step_counter += 1
                if area > 3500:
                    self.stop_robot()
                    return
                if self.step_counter >= 4:
                    self.search_step = 4 
                    self.step_counter = 0

            elif self.search_step == 4:
                twist.linear.x = 0.0
                twist.angular.z = -0.1
                self.publisher.publish(twist)
                time.sleep(1.0)
                self.step_counter += 1
                if area > 3500:
                    self.stop_robot()
                    return
                if self.step_counter >= 4:
                    self.search_step = 0 
                    self.step_counter = 0
                    self.cycle_count += 1
                    print(f"Cycle {self.cycle_count} completed.")
                
            if self.cycle_count >= 10:
                print("Search completed after 3 cycles. No green found.")
                self.searching_for_line = False
                self.stop_robot()
                return

        if self.should_move and self.ext_control:
            self.publisher.publish(message)
            self.running ="OK"
        else:
            empty_message = Twist()
            self.publisher.publish(empty_message)
    
    def stop_robot(self):
        """Stop all motion."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
    
    def crop_size(self,height, width):
        return (375,height,0,960)
        
    def get_contour_data(self,noise_yellow, out,mask):

        contours, _ = cv2.findContours(noise_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        mark = {}
        line = {}
        area = 0 
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            solidity = cv2.contourArea(contour) / (w * h)
            if y + h < mask.shape[0] * 0.6:
                continue
            #print(f"Aspect ratio: {aspect_ratio} and solidity: {solidity}")
            if aspect_ratio > 4.8 or aspect_ratio < 0.50:
                
                continue
            if solidity <= 0.10:
                
                continue
            #print("Area is: ", area)
            #if area >self.min_area:
            M = cv2.moments(contour)
            #print(f"Contour area: {M['m00']}")
            if M['m00'] > self.MIN_AREA:

                if (M['m00'] > self.MIN_AREA_TRACK):
                    # Contour is part of the track
                    line['x'] = crop_w_start + int(M["m10"]/M["m00"]) #crop_w_start
                    line['y'] = int(M["m01"]/M["m00"])
            
                        
        if 'x' in line and 'y' in line:
            if line['x'] > line['y']:
                mark_side = "right"
                # (line, mark_side)
            else:
                mark_side = "left"
        else:
            mark_side = None
        return (line, mark_side, area)
    
    def get_shape_data(self, crop_shape, output):
        
        threshold = cv2.adaptiveThreshold(crop_shape, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                 cv2.THRESH_BINARY, 11, 2)
        contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        i = 0
        for contour in contours:  
            if i == 0: 
                i = 1
                continue
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True) 
            cv2.drawContours(output, [contour], 0, (0, 0, 255), 5) 
            M = cv2.moments(contour) 
            if M['m00'] != 0.0: 
                x = int(M['m10']/M['m00']) 
                y = int(M['m01']/M['m00']) 
        
                # Check for quadrilateral (4 vertices)
                if len(approx) == 4:
                    # Compute the bounding box of the contour
                    x, y, w, h = cv2.boundingRect(approx)

                    # Compute the aspect ratio (width/height)
                    aspect_ratio = float(w) / h

                    # Check if the aspect ratio is reasonable for a quadrilateral (close to 1 for square, slightly more for rectangle)
                    if 0.8 <= aspect_ratio <= 1.2:
                        cv2.putText(output, 'Square', (x, y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        self.shape = 'Square'
                    else:
                        cv2.putText(output, 'Square', (x, y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        self.shape = 'Square'
                else:
                    self.shape = 'None'
        
        return self.shape



    
class Error_Check(Node):
    def __init__(self):
        super().__init__('System_Errors',namespace=robot_name)
        self.subscription = self.create_subscription(Image, '/'+robot_name+'/zed_node/rgb_gray/image_rect_gray',
                                             self.image_callback,
                                             qos_profile=rclpy.qos.qos_profile_system_default)
        self.publisher = self.create_publisher(Error, "/"+robot_name+"/system_hardware_error", 10)
        self.subscription
        self.system_run = False
        self.min_dBm = -100
        self.max_dBm = -30
        self.ret_message = ""

    def image_callback(self,data):
        message = Error()
        np_data = np.array(data.data, dtype=np.uint8)
        if isinstance(np_data, np.ndarray):
            message.camera = True
        else:
            message.camera = False #true is good, false is bad
        result_slam_lidar = subprocess.run(['ip', 'link', 'show', 'eqos0'], capture_output=True, text=True)
        result_estop_lidar = subprocess.run(['ip', 'link', 'show', 'eth0'], capture_output=True, text=True)
        result_wifi = subprocess.run(['iw', 'dev', 'wlan0', 'link'], capture_output=True, text=True, check=True)
        match = re.search(r'signal: (-\d+) dBm', result_wifi.stdout)
        if match:
            match_int = int(match.group(1))
            if match_int < self.min_dBm:
                match_int = self.min_dBm
            elif match_int > self.max_dBm:
                match_int = self.max_dBm
            message.wifi = (match_int - self.min_dBm) / (self.max_dBm - self.min_dBm) * 100
        else:
            message.wifi = 0.0
        if bool('state UP' in result_estop_lidar.stdout) and bool('state UP' in result_slam_lidar.stdout) and message.wifi > 10.0 and message.camera:
            
            self.system_run = True
            self.ret_message = "All systems are good !"
        else:
            self.system_run = False
            self.ret_message = " | ".join([
                                "E-Stop Lidar is DOWN" if 'state UP' not in result_estop_lidar.stdout else "",
                                "SLAM Lidar is DOWN" if 'state UP' not in result_slam_lidar.stdout else "",
                                "WiFi signal is weak" if message.wifi <= 10.0 else "",
                                "Camera is OFF" if not message.camera else ""
                            ]).replace(" |  | ", " | ").strip(" | ")
        message.estop_lidar = bool('state UP' in result_estop_lidar.stdout)
        message.slam_lidar = bool('state UP' in result_slam_lidar.stdout)

        self.publisher.publish(message)

class TakeContainer(Node):
    
    def __init__(self):
        super().__init__("take_cont")
        self.station_subscriber = self.create_subscription(String,"container_ready",self.station_feedback, 10)
        self.station_subscriber
        self.station_name = ""
        self.take_the_rack = False
    
    def station_feedback(self,msg):
        if self.station_name == msg.data.split(',')[0]:
            if msg.data.split(',')[1] == "Good":
                self.take_the_rack = True
            else:
                self.take_the_rack = False

class FieldSelectDyn(Node):
    def __init__(self):
        super().__init__("DynamicServiceCall",namespace=robot_name)
        self.ignore_select = -1
        self.field_select = 2 #1 is station, 2 is expanded for driving
        self.client = self.create_client(FieldSetWriteSrv,'/'+robot_name+'/FieldSetWrite')
        self.read_client = self.create_client(FieldSetReadSrv, f'/{robot_name}/FieldSetRead')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger("Waiting for service to be up")

        while not self.read_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for FieldSetRead service...")

    def read_current_field(self):
        request = FieldSetReadSrv.Request()
        future = self.read_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            current_field = response.active_field_set
            self.get_logger().info(f"Current active field: {current_field}")
            return current_field
        except Exception as e:
            self.get_logger().error(f"Failed to read current field: {str(e)}")
            return None
        
    def send_request(self):
        current_field = self.read_current_field()
        if current_field == self.field_select:
            self.get_logger().info("Field is already set. Skipping write.")
            return
        request = FieldSetWriteSrv.Request()
        request.field_set_selection_method_in = self.ignore_select
        request.active_field_set_in = self.field_select
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service call succeeded: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

def update_amr(db, amr):
    """Update AMR details in MongoDB."""
    
    update_doc = {
        "_id": amr.amr, 
        "errors": amr.errors,
        "totalcycles": amr.totalcycles,
        "batterystatus": amr.batterystatus,
        "stop": amr.stop,
        "station_curr": amr.station_curr,
        "mission_type": amr.mission_type,
        "generated_path": amr.generated_path,
        "amr": amr.amr,
        "current_pose":amr.current_pose
    }
    try:
        result = db.AMR.update_one(
            {"_id": amr.amr},
            {"$set": update_doc}
        )
        return result
    except Exception as e:
        logging.error(f"Error updating AMR {amr.amr}: {e}")
        return None
    
def update_station_one_field(db,station,field: str,robot):
    update_doc = {
        field: getattr(station, field)
    }
    try:
        return db.Stations.update_one(
            {"assigned": robot},
            {"$set": update_doc},
            upsert=True
        )
    except Exception as err:
        print("Error: {0}".format(err))
        return False

def first_boot_update(db, amr_number: int):
    try:
        db.AMR.update_one(
            {"_id": amr_number},
            {"$set": {
                "station_curr": "",
                "mission_type": "",
                "generated_path": ""
            }}
        )
        db.Stations.update_one(
            {"assigned": amr_number},
            {"$set": {
                "docking_in_action": False,
                "part_number": "",
                "assigned": 0
            }}
        )
        return

    except Exception as e:
        logging.error(f"Error updating AMR {amr_number} and Station: {e}")
        return None, None 

def read_amr(db, amr):
    try:
        amr_data = db.AMR.find_one({"_id": amr.amr})
        if amr_data:
            amr.errors = amr_data["errors"]
            amr.totalcycles = amr_data["totalcycles"]
            amr.batterystatus = amr_data["batterystatus"]
            amr.stop = amr_data["stop"]
            amr.station_curr = amr_data["station_curr"]
            amr.mission_type = amr_data["mission_type"]
            amr.generated_path = amr_data["generated_path"]
        return amr

    except Exception as e:
        logging.error(f"Error reading database for AMR {amr.amr}: {e}")
        return None
    
def read_station(db, station, robot):
    try:
        station_data = db.Stations.find_one({"assigned": robot})
        if station_data:
            station.cart_in_place = station_data["cart_in_place"]
            station.assigned = station_data["assigned"]
            station.docking_in_action = station_data["docking_in_action"]
            station.tag_id = station_data["tag_id"]
        return station

    except Exception as e:
        logging.error(f"Error reading database for AMR {station.station_name}: {e}")
        return None

def pickup_station(db, amr: AMR, mission_type=None):
    """Selects a station for the given AMR."""
    max_distance = 0
    station = None
    station_assigned = False
    param_name = "goal_vicinity_tol"
    node_name = "/"+robot_name+"/bt_navigator"

    try:
        if amr.mission_type not in ["Home","Unloader"]:
           
            cursor = db.Stations.find()
            for doc in cursor:
                if (doc["priority"] > max_distance and doc["assigned"] == 0 and
                    doc["cart_in_place"] is True and doc["production_enabled"] is True):
                    station = doc
                    station_assigned = True
                    mission_type = "Regular"
                    amr.station_curr = station["station_name"]  
                    amr.mission_type = "Regular"
                    
                    db.Stations.update_one(
                        {"_id": station["_id"]},
                        {"$set": {
                            "assigned": amr.amr 
                        }}
                    )
                    break
                elif (doc["priority"] > max_distance and doc["assigned"] == 0 and doc["production_enabled"] is True and doc["cart_in_place"] is False):
                    station = doc
                    station_assigned = True
                    mission_type = "Regular"
                    amr.station_curr = station["station_name"] 
                    amr.mission_type = "Regular" 
                    
                    db.Stations.update_one(
                        {"_id": station["_id"]},
                        {"$set": {
                            "assigned": amr.amr  
                        }}
                    )
                    break

            while station is None and not station_assigned:
                time.sleep(5)
                station = db.Stations.find_one({
                    "assigned": 0
                }, sort=[("priority", 1)])
                if station is not None:
                    
                    amr.station_curr = station["station_name"] 
                    amr.mission_type = "Regular"  
                    
                    db.Stations.update_one(
                        {"_id": station["_id"]},
                        {"$set": {
                            "assigned": amr.amr  
                        }}
                    )

        elif amr.mission_type == "Home":
           
            station = db.Dropoff.find_one({
                "dest": "Unloader_Home"
            })
            mission_type = amr.mission_type  

        elif amr.mission_type == "Unloader":
            
            station = db.Dropoff.find_one({
                "dest": amr.station_curr + "_Unloader"
            })
            mission_type = amr.mission_type  
            amr.station_curr = ""  
            amr.generated_path = ""  
            
            db.Stations.update_one(
                {"assigned": amr.amr},
                {"$set": {
                    "assigned": 0,
                    "docking_in_action": False
                }}
            )

        if station:
            if "station_name" in station:
                station = db.Stations.find_one({"_id": station["_id"]})
                result = subprocess.run(["ros2", "param", "set", node_name, param_name, "5.0"],
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                print("Output:", result.stdout)
            else:  # It's from the Dropoff table
                station = db.Dropoff.find_one({"_id": station["_id"]})
                result = subprocess.run(["ros2", "param", "set", node_name, param_name, "0.00"],
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                print("Output:", result.stdout)
    
    except Exception as e:
        logging.error(f"An error occurred in pickup_station: {e}")

    return station, mission_type, amr



def distance(point1, point2):
    """euclidean distance in terms of recovery"""
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def vector_from_points(p1, p2):
    """Return the vector from point p1 to point p2"""
    return (p2[0] - p1[0], p2[1] - p1[1])

def dot_product(v1, v2):
    """Return the dot product of vectors v1 and v2"""
    return v1[0] * v2[0] + v1[1] * v2[1]

def start_stop_line_follower(line_follower, mission_type, station, localization_check,amr,myDB_6,container_ready,battery,server,estop_sw,timeout,movement) -> str:
    if mission_type in ["Regular", "Unloader"]:
        if mission_type == "Regular":
            line_follower.initial_mission = "Station"
            line_follower.thres = 0.30
        elif mission_type == "Unloader":
            line_follower.initial_mission = "Unloader"
            line_follower.thres = 0.70
        line_follower.ext_control = True
        line_follower.start_stop_follower(command=True)
        delay_counter = 0
        processed =  False
        distance_travelled = 0.0
        one_time = True
        movement_threshold = movement
        exit_flag = False
        processed = False
        # pdb.set_trace()
        print("Starting line follower")
        temp_distance = (
                            localization_check.initial_pose.pose.pose.position.x,
                            localization_check.initial_pose.pose.pose.position.y
                        )
        #while line_follower.running != "OK":
            #print(f"mark_side={line_follower.mark_side}, error={line_follower.error}, angular_z={line_follower.angular_z}, average_pixels={line_follower.average_pixels}")
        #    time.sleep(1.0)
        timeout_threshold = timeout
        timeout_threshold_line = 180.0
        line_follower.station_release_charger = False
        
        start_time = time.time()
        while estop_sw.robot_mode == 'Auto' and not exit_flag:
            if estop_sw.global_estop:
                start_time = time.time()
            if line_follower.first_time:
                
                while line_follower.searching_for_line:
                    time.sleep(0.2)
                    if time.time() - start_time > timeout_threshold_line:
                        exit_flag = True
                        print("Timeout reached due to robot inactivity in searching the line! Exiting loop.")
                        break
            while not line_follower.searching_for_line and line_follower.first_time:
                elapsed_time_in_steady_state = time.time() - start_time
                if (line_follower.mark_side is None or line_follower.error == 0 or abs(line_follower.angular_z) == 0.0000) and line_follower.station_release_charger:
                    if elapsed_time_in_steady_state > timeout_threshold:
                        exit_flag = True
                        print("Timeout reached due to robot inactivity inside loop! Exiting loop.")
                        break
                else:
                    start_time = time.time()
                if line_follower.green_pixels > 9000 and one_time:
                    print(f"Distance travelled is : {distance_travelled}")
                    distance_travelled = distance(temp_distance,(localization_check.initial_pose.pose.pose.position.x,
                                localization_check.initial_pose.pose.pose.position.y))
                    if distance_travelled > movement_threshold:
                        print("Distance travelled is more than 2.0")
                        line_follower.first_time_angular = True
                        one_time = False
                
                if line_follower.average_pixels > 9000 and mission_type == "Regular" and not processed and not estop_sw.global_estop and distance_travelled > movement_threshold: #19125
                    #print(line_follower.ext_control)

                    delay_counter += 1
                    
                    if delay_counter == 1:
                        battery.robot_charge_cmd = True
                        #line_follower.ext_control = False
                        initial_position = (
                            localization_check.initial_pose.pose.pose.position.x,
                            localization_check.initial_pose.pose.pose.position.y
                        )
                    time.sleep(float(station['charge_min_time']))
                    if not container_ready.take_the_rack:
                        line_follower.stop_robot()
                        print(f"waiting for cart plus charging, current soc is : {battery.battery_soc}")
                        time.sleep(0.5)
                        start_time = time.time()
                    if delay_counter > 2 and container_ready.take_the_rack: 
                        battery.robot_charge_cmd = False
                        line_follower.robot_tunnel_vision = True
                        time.sleep(1)
                        line_follower.station_release_charger = True
                        line_follower.ext_control = True
                        processed = True
                        
                        #line_follower.extension_control = True
                elif line_follower.average_pixels > 5500 and mission_type == "Regular" and processed and not estop_sw.global_estop and distance_travelled > movement_threshold:
                    current_position = (
                        localization_check.initial_pose.pose.pose.position.x,
                        localization_check.initial_pose.pose.pose.position.y
                        )
                    distance_temp = distance(initial_position, current_position)
                    print(f"distance_temp = {distance_temp}")
                    if distance_temp >= movement_threshold and distance_temp != 0.0:
                        exit_flag = True
                        break

                
                elif line_follower.average_pixels > 12000 and mission_type == "Unloader" and not estop_sw.global_estop and distance_travelled > movement_threshold:
                    delay_counter += 1
                    battery.unloader = True
                    if delay_counter == 1:
                        amr.station_curr = mission_type
                        line_follower.ext_control = False
                        update_amr(myDB_6, amr)
                        initial_position = (
                        localization_check.initial_pose.pose.pose.position.x,
                        localization_check.initial_pose.pose.pose.position.y
                        )
                        server.unloader_cmd = False
                        time.sleep(1)
                        

                    while amr.station_curr == "Unloader":
                        
                        time.sleep(1)
                        start_time = time.time()
                        #user_input = input("Press 'q' to continue, mimicing server")
                        #if user_input.lower() == 'q': 
                        if server.unloader_cmd:
                            line_follower.ext_control = True
                            print("Going to forklift.")
                            amr.station_curr = ""
                            update_amr(myDB_6, amr)
                            battery.unloader = False
                            break

                    if delay_counter > 2 and amr.station_curr == "":
                        line_follower.ext_control = True
                        server.forklift_cmd = False
                        #line_follower.extension_control = True
                        
                        print("Going to Forklift now")
                        current_position = (
                        localization_check.initial_pose.pose.pose.position.x,
                        localization_check.initial_pose.pose.pose.position.y
                        )
                        distance_temp = distance(initial_position, current_position)
                        if line_follower.average_pixels > 15000 and  distance_temp >= movement_threshold and distance_temp != 0.0 and not estop_sw.global_estop:
                            battery.forklift = True
                            server.forklift_cmd = False
                            #pdb.set_trace()
                            print(distance_temp)
                            line_follower.ext_control = False
                            #line_follower.extension_control = False
                            amr.station_curr = "Forklift"
                            amr.mission_type = "Forklift"
                            update_amr(myDB_6, amr)
                            mission_type = "Forklift"

                    while amr.station_curr == "Forklift":
                        start_time = time.time()
                        server.forklift_cmd = False
                        time.sleep(0.5)
                        #user_input = input("Press 'q' to continue, mimicing server")
                        #if user_input.lower() == 'q':
                        if server.forklift_cmd:
                            battery.unloader = True
                            print("Going back to home.")
                            amr.station_curr = ""
                            update_amr(myDB_6, amr)
                            exit_flag = True
                            battery.forklift = False
                            break
                    if exit_flag:
                        break
                time.sleep(0.01)
                print(f"mark_side={line_follower.mark_side}, error={line_follower.error}, angular_z={line_follower.angular_z}")
                print(f"average_pixels red ={line_follower.average_pixels} and green = {line_follower.green_pixels}")
    line_follower.ext_control = False
    line_follower.station_release_charger = False
    print("Ending line follower")
    line_follower.start_stop_follower(command=False)
    if mission_type == "Regular":
        time.sleep(1.0)
        print("Going to Unloader")
        
        amr.mission_type = 'Unloader'
        mission_type = 'Unloader'
        update_amr(myDB_6, amr)
        return mission_type

    elif mission_type == "Forklift": #localization_check.tag_id == station.tag_id and
        time.sleep(1.0)
        amr.mission_type = 'Home'
        mission_type = 'Home'
        update_amr(myDB_6, amr)
        return mission_type
    
    elif estop_sw == 'Manual':
        return mission_type

    return mission_type

    
def handle_failed_task(robot, localization_check, container_ready, server, battery,navigator, pin, loop_counter, line_follower, recovery, executor, footprint_change,myDB_6,amr,inter_proj_6, mission_type,estop_sw,myDB_3,driving_inter,inter_clearing_dist):
    """Reusable recovery in failure mode (10 times loop X 1 minute interval on failure + BT tree 10000 times recovery should be redundant before human intervention)"""

    while loop_counter < 10:
        loop_counter += 1
        if amr.generated_path is not None:
            poses_data = json.loads(amr.generated_path)

            min_distance = float('inf')
            closest_pose_index = None
            current_robot_location = (
                localization_check.initial_pose.pose.pose.position.x,
                localization_check.initial_pose.pose.pose.position.y
            )

            w = localization_check.initial_pose.pose.pose.orientation.w
            x = localization_check.initial_pose.pose.pose.orientation.x
            y = localization_check.initial_pose.pose.pose.orientation.y
            z = localization_check.initial_pose.pose.pose.orientation.z

            # Calculate yaw
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)

            # Find closest pose
            for i, pose_data in enumerate(poses_data):
                pose_location = (float(pose_data['x']), float(pose_data['y']))
                dist = distance(current_robot_location, pose_location)

                if dist < min_distance and abs(yaw_z) < 1.04:  # Use a constant for orient_angle
                    min_distance = dist
                    closest_pose_index = i

            poses_from_closest_to_end = poses_data[closest_pose_index:]
        else:
            navigator.clearCostmapAroundRobot(2.0)
            station, mission_type, amr = pickup_station(myDB_6,amr,mission_type)
            poses_from_closest_to_end = station['path']

        # Prepare navigation poses
        navigate_poses = []
        navigator.clearCostmapAroundRobot(2.0)
        
        for item in poses_from_closest_to_end:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(item['x'])
            goal_pose.pose.position.y = float(item['y'])
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = float(item['z'])
            goal_pose.pose.orientation.w = float(item['w'])
            navigate_poses.append(goal_pose)
            time.sleep(0.05)

        # Get path correctly
        if len(navigate_poses) > 1:
            path = navigator.getPathThroughPoses(navigate_poses)
        elif len(navigate_poses) == 1:
            path = navigator.getPath(navigate_poses[0], navigate_poses)  # Pass the single goal pose
        
        if path is not None:
            
            intersection_point = check_intersections_in_path(path, inter_proj_6)
            poses_data = [{'x': "{:.2f}".format(pose_stamped.pose.position.x), 
                            'y': "{:.2f}".format(pose_stamped.pose.position.y), 
                            'z': "{:.2f}".format(pose_stamped.pose.orientation.z), 
                            'w': "{:.2f}".format(pose_stamped.pose.orientation.w)}
                           for pose_stamped in path.poses]

            amr.generated_path = json.dumps(poses_data)
            update_amr(myDB_6,amr)  # Update AMR details 
            # Update Station details

            navigator.goThroughPoses(navigate_poses)
            i = 0
            while not navigator.isTaskComplete():
                i += 1
                feedback = navigator.getFeedback()
                
                if feedback and i % 5 == 0:
                    current_robot_location = (
                        localization_check.initial_pose.pose.pose.position.x,
                        localization_check.initial_pose.pose.pose.position.y
                    )
                    amr.current_pose = [{"x": str(current_robot_location[0]), "y": str(current_robot_location[1])}]
                    update_amr(myDB_6,amr)
                    if intersection_point:
                        inter_point = (float(intersection_point[0].inter_point.x), float(intersection_point[0].inter_point.y))
                        distance_to_inter = distance(current_robot_location, inter_point)
                        print(f'Distance remaining from intersection: {distance_to_inter} meters.')
                        if distance_to_inter <= intersection_point[0].stop_dist:
                            occupied_status_3, update_id = check_occupancy(intersection_point, myDB_3)
                            occupied_status_6 = bool(intersection_point.occupied)
                            if (occupied_status_3 or occupied_status_6) and driving_inter is False:
                                estop_sw.not_clear = True
                                time.sleep(random.uniform(1, 2))
                                print("Waiting for intersection to be cleared")
                                if occupied_status_3 and not occupied_status_6:
                                    myDB_6.Inters.update_one(
                                        {"_id": intersection_point._id},
                                        {"$set": {"occupied": True}}
                                    )
                                elif occupied_status_6 and not occupied_status_3:
                                    myDB_3.intersections.update_one(
                                        {"_id": update_id},
                                        {"$set": {"occupied": True}}
                                    )
                            else:
                                driving_inter = True
                                estop_sw.not_clear = False
                                myDB_6.Inters.update_one(
                                    {"_id": intersection_point._id},
                                    {"$set": {"occupied": True}}
                                )
                                myDB_3.intersections.update_one(
                                    {"_id": update_id},
                                    {"$set": {"occupied": True}}
                                )
                                if driving_inter:
                                    temp = +1
                                    if temp == 1:
                                        distance_at_inter = distance_remaining
                                    distance_travelled_since_inter = distance_at_inter - distance_remaining

                        elif driving_inter and distance_travelled_since_inter >= inter_clearing_dist:

                            myDB_6.Inters.update_one(
                                {"_id": intersection_point._id},
                                {"$set": {"occupied": False}}
                            )
                            myDB_3.intersections.update_one(
                                {"_id": update_id},
                                {"$set": {"occupied": False}}
                            )
                            temp = 0
                            intersection_point.pop(0)
                            driving_inter = False

                    else:
                        print("No intersection found")
                    eta_seconds = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                    distance_remaining = '{:.2f}'.format(feedback.distance_remaining)
                    print(f'Estimated time of arrival: {eta_seconds:.0f} seconds.')
                    print(f'Distance remaining: {distance_remaining} meters.')
                    if amr.stop:
                        navigator.cancelTask()
                        print("User cancelled from database")

            # Check task result
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:

                print('Goal reached.')
                
                print(mission_type)
                if mission_type == "Regular":
                    try:
                        executor.add_node(line_follower)
                        executor.add_node(container_ready)
                        footprint_change.decrease_footprint = False
                        container_ready.station_name = station["station_name"]
                        pin.pinValue = False
                        time.sleep(1.0)
                        line_follower.LINEAR_SPEED = 0.07
                        #estop_sw.lf_oper = True
                        result_line = start_stop_line_follower(line_follower, mission_type, station, localization_check, amr, myDB_6,container_ready,battery,server)
                        estop_sw.lf_oper = False
                        pin.pinValue = True
                    finally:
                        executor.remove_node(line_follower)
                        executor.remove_node(container_ready)
                    mission_type = result_line
                elif mission_type == "Unloader":
                    
                    try:
                        executor.add_node(server)
                        executor.add_node(line_follower)
                        footprint_change.decrease_footprint = False
                        time.sleep(1.0)
                        line_follower.LINEAR_SPEED = 0.15
                        #estop_sw.lf_oper = True
                        result_line = start_stop_line_follower(line_follower, mission_type, station, localization_check, amr, myDB_6,container_ready,battery,server)
                        estop_sw.lf_oper = False
                    finally:
                        executor.remove_node(line_follower)
                        executor.remove_node(server)
                elif mission_type == "Home":
                    mission_type = "Regular"
                    amr.mission_type = mission_type
                    amr.station_curr = ""
                    
            elif result == TaskResult.CANCELED:
                print('Task was canceled by user intervention. Please press Retry!')
                break
            elif result in [TaskResult.FAILED, TaskResult.UNKNOWN]:
                print("Retrying task due to failure.")
                time.sleep(0.5)

def read_intersection_data_3(db):
    """Load all intersections from the database into a list of Inter_Porj_3 dataclass instances."""
    intersections_data = db.intersections.find()
    intersections = []
    for intersection_data in intersections_data:
        intersections.append(Inter_Porj_3(
            id=intersection_data['_id'],
            ins=intersection_data['ins'],
            out=intersection_data['out'],
            priority= intersection_data['priority'],
            occupied=intersection_data['occupied']
        ))
    return intersections

def read_intersection_data_6(db):
    """Load all intersections from the database into a list of Inter_Porj_6 dataclass instances."""
    intersections_data = db.Inters.find()
    intersections = []
    for intersection_data in intersections_data:
        intersections.append(Inter_Porj_6(
            id=intersection_data['_id'],
            rfid_in=intersection_data['rfid_in'],
            occupied=intersection_data['occupied'],
            stop_dist=intersection_data['stop_dist'],
            inter_point_in=intersection_data['inter_point_in'],
            inter_point_out=intersection_data['inter_point_out']
        ))
    return intersections

def check_intersections_in_path(path, inter_proj_6,tolerance=0.05):
    """Check if any intersection points exist in the given path and return a list of intersections."""
    found_intersections = []
    for point in path.poses:
        current_point = (point.pose.position.x, point.pose.position.y)
        for intersection in inter_proj_6:
            inter_points_list = json.loads(intersection.inter_point_in)
            for inter_point_dict in inter_points_list:
                inter_point = (inter_point_dict["x"], inter_point_dict["y"])
                distance = math.sqrt((current_point[0] - inter_point[0])**2 + (current_point[1] - inter_point[1])**2)
                if distance <= tolerance:
                    found_intersections.append(intersection)
                    break
    return found_intersections

def update_intersection(db, intersection):
    """Update the intersection in the database."""
    db.Intersections.update_one(
        {"inter_points": intersection.inter_points},
        {"$set": intersection.__dict__}
    )

def update_intersection_status(db, rfid, new_status):
    """Update the intersection's occupied status in the database."""
    db.intersections.update_one(
        {"ins": {"$in": [rfid]}},
        {"$set": {"occupied": new_status}}
    )

def check_occupancy(intersection_point, db):
    rfid_in_list = intersection_point[0].rfid_in
    intersections = read_intersection_data_3(db)
    for intersection in intersections:
        if intersection.occupied and any(rfid in intersection.ins for rfid in rfid_in_list):
            logging.info(f"Intersection {intersection.id} is occupied.")
            return True,intersections._id
    logging.info("No intersections are occupied.")
    return False,None



robot_name = (os.getenv('ROBOT_MODEL','amr_x'))
robot = int(robot_name.split('_')[1])

def main():
    """Global settings"""

    path_counter = 0
    mission_type=None
    recovery=False
    distance_at_inter = 0.0
    distance_travelled_since_inter = 0.0
    inter_clearing_dist = 1.0
    driving_inter = False
    timeout = 5.0
    max_retries = 10
    retry_count = 0
    movement = 0.0

    myDB_6 = get_db("project6")
    myDB_3 = get_db("project3")
    first_boot_update(myDB_6, robot)
    amr = AMR(
        amr=robot,
        errors='',
        totalcycles=0,
        batterystatus=0.0,
        stop=False,
        station_curr='',
        mission_type='',
        generated_path='',
        current_pose=''
    )

    inter_proj_3: List[Inter_Porj_3] = []
    inter_proj_6: List[Inter_Porj_6] = []


    read_amr(myDB_6, amr)
    inter_proj_6 = read_intersection_data_6(myDB_6)
    inter_proj_3 = read_intersection_data_3(myDB_3)
    rclpy.init(args=None)

    try:
        navigator = BasicNavigator(namespace=robot_name)
        localization_check = LocalizationCheck()
        estop_sw = SW_ESTOP()
        pin = PinPublisher(estop_sw)
        container_ready = TakeContainer()
        server = ServerConnect()
        error_check = Error_Check()
        lidar_switch_field = FieldSelectDyn()
        battery = ChargerCMD()
        line_follower = Linefollower()
        footprint_change = ChangeFootprint()
        executor = MultiThreadedExecutor()
        executor.add_node(localization_check)
        executor.add_node(estop_sw)
        executor.add_node(battery)
        executor.add_node(lidar_switch_field)
        executor.add_node(error_check)
        executor.add_node(pin)
        #executor.add_node(line_follower)
    
        executor_try = Thread(target=executor.spin,daemon=True)
        executor_try.start()
        
        while rclpy.ok() and retry_count < max_retries:
            if robot > 0:
                if localization_check.localization and not pin.initial_reset:
                    navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='')
                    estop_sw.lf_oper = False
                    lidar_switch_field.field_select = 1
                    lidar_switch_field.send_request()
                    amr.batterystatus = float(battery.battery_soc)
                    while error_check.system_run:
                        if estop_sw.robot_mode == 'Auto':
                            if not footprint_change in executor._nodes:
                                executor.add_node(footprint_change)
                            footprint_change.decrease_footprint = True
                            navigator.clearAllCostmaps()
                            executor.remove_node(footprint_change)
                            #navigator.clearglobalCostmapAroundRobot(10.0)
                            #navigator.clearlocalCostmapAroundRobot(10.0)
                            lidar_switch_field.field_select = 2
                            lidar_switch_field.send_request()
                            if retry_count == 0:
                                station, mission_type, amr = pickup_station(myDB_6,amr,mission_type)
                                goal_poses = []
                                navigate_poses = []
                                poses_data = json.loads(station['path'])
                            
                            if len(poses_data) > 1:
                                for item in poses_data:
                                    goal_pose = PoseStamped()
                                    goal_pose.header.frame_id = 'map'
                                    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                                    goal_pose.pose.position.x = float(item['x'])
                                    goal_pose.pose.position.y = float(item['y'])
                                    goal_pose.pose.position.z = 0.0
                                    goal_pose.pose.orientation.x = 0.0
                                    goal_pose.pose.orientation.y = 0.0
                                    goal_pose.pose.orientation.z = float(item['z'])
                                    goal_pose.pose.orientation.w = float(item['w'])
                                    navigate_poses.append(goal_pose)

                                current_pose = PoseStamped()
                                current_pose.header.frame_id = 'map'
                                current_pose.header.stamp = navigator.get_clock().now().to_msg()
                                current_pose.pose.position.x = localization_check.initial_pose.pose.pose.position.x
                                current_pose.pose.position.y = localization_check.initial_pose.pose.pose.position.y
                                current_pose.pose.position.z = 0.0
                                current_pose.pose.orientation.x = 0.00
                                current_pose.pose.orientation.y = 0.00
                                current_pose.pose.orientation.z = localization_check.initial_pose.pose.pose.orientation.z
                                current_pose.pose.orientation.w = localization_check.initial_pose.pose.pose.orientation.w
                                time.sleep(0.5)
                                path = navigator.getPathThroughPoses(current_pose, navigate_poses)
                                time.sleep(5.0)
                                #pdb.set_trace()
                                if path is None:
                                    for path_counter in range(20):
                                        navigator.clearglobalCostmapAroundRobot(5.0+path_counter)
                                        navigator.clearlocalCostmapAroundRobot(5.0+path_counter)
                                        time.sleep(2.0)
                                        path = navigator.getPathThroughPoses(current_pose, navigate_poses)
                                        if path is None and path_counter == 10 or path_counter == 20:
                                            navigator.clearAllCostmaps()
                                        if path is not None:
                                            break
                                pin.pinValue = True
                                time.sleep(0.5)
                                intersection_point = check_intersections_in_path(path, inter_proj_6)
                                poses_data = []
                                for pose_stamped in path.poses:
                                    poses_data.append({
                                        "x": "{:.2f}".format(pose_stamped.pose.position.x),
                                        "y": "{:.2f}".format(pose_stamped.pose.position.y),
                                        "z": "{:.2f}".format(pose_stamped.pose.orientation.z),
                                        "w": "{:.2f}".format(pose_stamped.pose.orientation.w)
                                    })
                                amr.generated_path = json.dumps(poses_data)
                                amr.mission_type = mission_type
                                update_amr(myDB_6,amr)
                                
                                navigator.goThroughPoses(navigate_poses)
                                i = 0
                                x = 0
                                while not navigator.isTaskComplete() and estop_sw.robot_mode == 'Auto':
                                    i += 1
                                    feedback = navigator.getFeedback()
                                    
                                    if feedback and i % 5 == 0:
                                        current_robot_location = (
                                            localization_check.initial_pose.pose.pose.position.x,
                                            localization_check.initial_pose.pose.pose.position.y
                                        )
                                        amr.current_pose = [{"x": str(current_robot_location[0]), "y": str(current_robot_location[1])}]
                                        amr.batterystatus = battery.battery_soc
                                        update_amr(myDB_6,amr)
                                        if intersection_point:
                                            try:
                                                # Load intersection points
                                                inter_point_dict = json.loads(intersection_point[0].inter_point_in)[0]
                                                inter_point_dict_out = json.loads(intersection_point[0].inter_point_out)[0]
                                                inter_point = (float(inter_point_dict["x"]), float(inter_point_dict["y"]))
                                                inter_point_out = (float(inter_point_dict_out["x"]), float(inter_point_dict_out["y"]))

                                                # Calculate vectors and distances
                                                vec_to_inter = vector_from_points(current_robot_location, inter_point)
                                                vec_to_inter_out = vector_from_points(inter_point, inter_point_out)
                                                dot_to_inter = dot_product(vec_to_inter, vec_to_inter_out)
                                                distance_to_inter = distance(current_robot_location, inter_point)
                                                distance_to_inter_out = distance(current_robot_location, inter_point_out)

                                                print(f"Distance to intersection: {distance_to_inter:.2f} meters.")
                                                print(f"Distance to exit: {distance_to_inter_out:.2f} meters.")
                                                if distance_to_inter <= intersection_point[0].stop_dist and dot_to_inter > 0:

                                                    # Fetch occupancy status from both databases
                                                    existing_status_3 = myDB_3.intersections.find_one({"ins": intersection_point[0].rfid_in})
                                                    existing_status_6 = myDB_6.Inters.find_one({"_id": intersection_point[0].id})
                                                    occupied_status_3 = bool(existing_status_3 and existing_status_3["occupied"])  # Other robot
                                                    occupied_status_6 = bool(existing_status_6 and existing_status_6["occupied"])  # This robot

                                                    print(f"Other robot occupied: {occupied_status_3}, This robot occupied: {occupied_status_6}")

                                                    # **If Both Robots Are in the Intersection and This Robot is Not Moving ? STOP**
                                                    if occupied_status_3 and not driving_inter:
                                                        print("Both robots have occupied the intersection. Waiting...")
                                                        estop_sw.not_clear = True  # Stop robot
        
                                                    else:
                                                        # **Occupy intersection for this robot**
                                                        print("No other robot detected. Occupying intersection and proceeding.")
                                                        myDB_6.Inters.update_one({"_id": intersection_point[0].id}, {"$set": {"occupied": True}})
                                                        myDB_3.intersections.update_one({"ins": intersection_point[0].rfid_in}, {"$set": {"occupied": True, "priority": True}})
                                                        estop_sw.not_clear = False  # Allow movement
                                                        driving_inter = True

                                                elif distance_to_inter_out <= intersection_point[0].stop_dist and dot_to_inter < 0:
                                                    # **Release intersection when exiting**
                                                    print("Preparing to release intersection...")

                                                    # **Double check occupancy before clearing**
                                                    existing_status_3 = myDB_3.intersections.find_one({"ins": intersection_point[0].rfid_in})
                                                    existing_status_6 = myDB_6.Inters.find_one({"_id": intersection_point[0].id})

                                                    
                                                    myDB_6.Inters.update_one({"_id": intersection_point[0].id}, {"$set": {"occupied": False}})
                                                    myDB_3.intersections.update_one({"ins": intersection_point[0].rfid_in}, {"$set": {"occupied": False, "priority": False}})
                                                    print("Intersection cleared.")

                                                    intersection_point.pop(0)
                                                    driving_inter = False

                                            except Exception as e:
                                                logging.error(f"Error in intersection handling: {e}")

                                        else:
                                            logging.info("No intersection found")
                                        eta_seconds = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                                        distance_remaining = '{:.2f}'.format(feedback.distance_remaining)
                                        logging.info(f'Estimated time of arrival at goal: {eta_seconds:.0f} seconds, where mission is: {mission_type}')
                                        print(f'Total Distance remaining: {distance_remaining} meters.')
                                        if amr.mission_type == "Unloader" and float(feedback.distance_remaining) < 2.0 and x == 0:
                                            lidar_switch_field.field_select = 3
                                            lidar_switch_field.send_request()
                                            x += 1
                                        if amr.stop:
                                            navigator.cancelTask()
                                            first_boot_update(myDB_6, robot)
                                            print("User cancelled using joystick")
                                result = navigator.getResult()
                                if estop_sw.robot_mode == 'Auto':
                                    if result == TaskResult.SUCCEEDED:
                                        estop_sw.lf_oper = True
                                        print('Goal reached.')
                                        print(mission_type)
                                        if mission_type == "Regular":
                                            try:
                                                movement = 2.0
                                                lidar_switch_field.field_select = 1
                                                lidar_switch_field.send_request()
                                                if line_follower not in executor._nodes:
                                                    executor.add_node(line_follower)
                                                if container_ready not in executor._nodes:
                                                    executor.add_node(container_ready)
                                                #footprint_change.decrease_footprint = False
                                                line_follower.initial_reset()
                                                container_ready.station_name = station["station_name"]
                                                pin.pinValue = False
                                                
                                                
                                                time.sleep(1.0)
                                                #pdb.set_trace()

                                                #cmd = ['ros2', 'param', 'set', '/'+robot_name+'/zed_node', 'video.exposure', str(90)]
                                                #result = subprocess.run(cmd, capture_output=True, text=True)
                                                #print("STDOUT:", result.stdout)
                                                line_follower.LINEAR_SPEED = 0.07
                                                timeout = 60.0
                                                #estop_sw.lf_oper = True
                                                result_line = start_stop_line_follower(line_follower, mission_type, station, localization_check, amr, myDB_6,container_ready,battery,server,estop_sw,timeout,movement)

                                                pin.pinValue = True
                                                #cmd = ['ros2', 'param', 'set', '/'+robot_name+'/zed_node', 'video.exposure', str(65)]
                                                #result = subprocess.run(cmd, capture_output=True, text=True)
                                                #print("STDOUT:", result.stdout)
                                                retry_count = 0
                                            finally:
                                                executor.remove_node(line_follower)
                                                executor.remove_node(container_ready)
                                            mission_type = result_line
                                        elif mission_type == "Unloader":
                                            
                                            try:
                                                movement = 2.0
                                                if server not in executor._nodes:
                                                    executor.add_node(server)
                                                if line_follower not in executor._nodes:
                                                    executor.add_node(line_follower)
                                                #footprint_change.decrease_footprint = False
                                                line_follower.initial_reset()
                                                
                                                time.sleep(0.5)
                                                timeout = 100000000000.0
                                                line_follower.LINEAR_SPEED = 0.10
                                                
                                                retry_count = 0
                                                result_line = start_stop_line_follower(line_follower, mission_type, station, localization_check, amr, myDB_6,container_ready,battery,server,estop_sw,timeout,movement)
                                                amr.totalcycles +=1
                                            finally:
                                                executor.remove_node(line_follower)
                                                executor.remove_node(server)
                                       
                                        elif mission_type == "Home":
                                            mission_type = "Regular"
                                            amr.mission_type = mission_type
                                            amr.station_curr = ""
                                            
                                    elif result == TaskResult.CANCELED:
                                        print('Task was canceled by user intervention. Please press Retry!')
                                    elif result in [TaskResult.FAILED, TaskResult.UNKNOWN]:
                                        recovery = True
                                        retry_count = +1
                                        print("Nav2 failed navigation")
                                        if retry_count > max_retries:
                                            print("Max retries reached. Exiting...")
                                            break
                                        print("Retrying task due to failure.")
                                        #handle_failed_task(robot, localization_check, navigator,inter_proj_6, pin, loop_counter, line_follower, recovery, executor, footprint_change, amr, myDB_6, container_ready, battery,server)
                                    estop_sw.lf_oper = False
                        elif estop_sw.robot_mode == 'Manual':  
                            if not navigator.isTaskComplete():
                                navigator.cancelTask()
                                first_boot_update(myDB_6, robot)
                            time.sleep(2)

                        else:
                            print("Unknown robot mode. Stopping...")
                            break 
                    print(f"System is still booting, waiting on the {error_check.ret_message}")
                    time.sleep(1)
                else:
                    print("Waiting for robot to be localized!")
                    time.sleep(5)
                    navigator.cancelTask()
            else:
                logging.info("The hostname of the robot is not set. Aborting program.")
                time.sleep(5)
                #sys.exit()
        


    except (KeyboardInterrupt,SystemError,SystemExit,SyntaxError) as e:
        lidar_switch_field.field_select = 1
        navigator.cancelTask()
        lidar_switch_field.send_request()
        time.sleep(5.0)
        logging.error(f"Exception occured: {e}")
        logging.error(traceback.format_exc())
        #rclpy.try_shutdown()
        #executor.remove_node(line_follower)
        #executor.remove_node(localization_check)
        #executor.remove_node(battery)
        #executor.remove_node(pin)
        #executor.remove_node(footprint_change
        navigator.lifecycleShutdown()
        navigator.destroy_node()
        executor.shutdown()

    finally:
        rclpy.shutdown()
        #sys.exit()
        
if __name__ == '__main__':
    main()