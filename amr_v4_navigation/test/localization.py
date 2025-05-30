#!/usr/bin/env python3
import time, os , pyodbc , rclpy, rclpy.qos, math, re
from enum import Enum, IntEnum

#from action_msgs.msg import GoalStatus
#from std_msgs.msg import Float64MultiArray
#from builtin_interfaces.msg import Duration
#from lifecycle_msgs.srv import GetState
#from nav2_msgs.action import BackUp, Spin
#from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
#from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose
#from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from enum import Enum

from rtabmap_msgs.msg import Info
from amr_v4_msgs_srvs.msg import Robot

#line follower
from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Point
from std_srvs.srv import Empty
import numpy as np
import cv2, cv_bridge
from cv_bridge import CvBridgeError
from matplotlib import pyplot as plt 
from pymongo import MongoClient
from bson.objectid import ObjectId
import subprocess
import re
import psutil
import asyncio
from jtop import jtop
'''
Global
Variables
'''


class LocalizationCheck(Node):
    """
    Checks if we are localized in home to continue to Navigation
    """
    global localization

    def __init__(self):
        super().__init__("localize_info")
        self.recieved_message = False
        self.publisher_ = self.create_publisher(Robot, "/amr/diagnostic", 10)
        self.subscriber_ = self.create_subscription(Info, "/info", self.callback_localize, 10)    
        self.timer = self.create_timer(2.0, self.callback_information)
        self.subscriber_
        self.localization = False

    def callback_localize(self,msg):
        if (msg.header.frame_id != None and msg.loop_closure_id != None and msg.landmark_id != 0):
            self.recieved_message = True
        self.get_logger().info('I heard you: "%s"' % msg.header.frame_id)   

    def callback_information(self):
        message = Robot()
        if (self.recieved_message):
            message.robot_localization_status = 'The Robot is Localized and Starting to Navigate !'
            message.localized = True
            self.localization = True
        else:
            message.robot_localization_status = 'Please drive the Robot to the Tag at HOME !'
            message.localized = False
            self.localization = False
        self.publisher_.publish(message)
        
def get_signal_strength(interface):
    try:
        # Run the iw command to get the current link information
        result = subprocess.run(['iw', 'dev', interface, 'link'], capture_output=True, text=True, check=True)
        
        # Extract the signal strength from the output using a regular expression
        match = re.search(r'signal: (-\d+) dBm', result.stdout)
        print(match.group(1))
        if match:
            return int(match.group(1))
        else:
            raise ValueError("Signal strength not found in output.")
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Error running iw command: {e}")
    except Exception as e:
        raise RuntimeError(f"An error occurred: {e}")

def dBm_to_percentage(signal_dBm, min_dBm=-100, max_dBm=-30):
    if signal_dBm < min_dBm:
        signal_dBm = min_dBm
    elif signal_dBm > max_dBm:
        signal_dBm = max_dBm

    percentage = (signal_dBm - min_dBm) / (max_dBm - min_dBm) * 100
    return percentage


def main(args=None):
    #myclient = MongoClient("mongodb://mrumel:56.583fudge@10.10.1.45:27017/?authSource=admin")
    #mydb = myclient["project3"]
    #collist = mydb.intersections.find_one({'ins':1240})
    #if collist is not None:
    #    print(collist['occupied']) 
    # min_dBm =-100
    # max_dBm =-30
    # result_wifi = subprocess.run(['iw', 'dev', 'wlan0', 'link'], capture_output=True, text=True, check=True)
    # match = re.search(r'signal: (-\d+) dBm', result_wifi.stdout)
    
    # if match:
    #     match_int = int(match.group(1))
    #     if match_int < min_dBm:
    #         match_int = min_dBm
    #     elif match_int > max_dBm:
    #         match_int = max_dBm
    #     wifi = (match_int - min_dBm) / (max_dBm - min_dBm) * 100
    #     print(wifi)
    with jtop() as jetson:
        # jetson.ok() will provide the proper update frequency
        while jetson.ok():
            print("Print fan status")
            print(jetson.fan)
            # Print for each fan all info
            for name in jetson.fan:
                speed = jetson.fan.get_speed(name)
                profile = jetson.fan.get_profile(name)
                print("{name} profile={profile} speed={speed}".format(name=name, profile=profile, speed=speed))
if __name__ == '__main__':
    main()