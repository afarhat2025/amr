#!/usr/bin/env python3 

import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import BatteryState
 
class BatteryStatePublisher(Node):
  """
  The class publishes the battery state at a specific time interval.
  """
  
  def __init__(self):
    super().__init__('battery_state_pub')
    self.publisher_battery_state = self.create_publisher(BatteryState, '/battery_status', 10)
    self.timer = self.create_timer(5.0, self.get_battery_state)
    self.battery_voltage = 9.0
    self.percent_charge_level = 1.0
    self.decrement_factor = 0.99
     
  def get_battery_state(self):
    msg = BatteryState()
    msg.voltage = self.battery_voltage 
    msg.percentage = self.percent_charge_level
    self.publisher_battery_state.publish(msg)
    #add communication with BMS here, this is ROS2 platform


   
def main(args=None):
  rclpy.init(args=args)
  battery_state_pub = BatteryStatePublisher()
  rclpy.spin(battery_state_pub)
  battery_state_pub.destroy_node()
 
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()