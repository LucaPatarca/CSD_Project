#!/usr/bin/env python3 
 
"""
Description:
Publish the battery state at a specific time interval
-------
Publishing Topics:
/battery_status – sensor_msgs/BatteryState
-------
Subscription Topics:
None
-------
Author: Addison Sears-Collins
Website: AutomaticAddison.com
Date: November 10, 2021
"""
  
import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class
from sensor_msgs.msg import BatteryState # Enable use of the sensor_msgs/BatteryState message type
from std_msgs.msg import String
  
class BatteryStatePublisher(Node):
  """
  Create a BatteryStatePublisher class, which is a subclass of the Node class.
  The class publishes the battery state of an object at a specific time interval.
  """
   
  def __init__(self):
    """
    Class constructor to set up the node
    """
    
    # Initiate the Node class's constructor and give it a name
    super().__init__('battery_state_pub')
      
    # Create publisher(s)  
      
    # This node publishes the state of the battery.
    # Maximum queue size of 10. 
    self.publisher_battery_state = self.create_publisher(BatteryState, 'battery_status', 10)
    self.subscription = self.create_subscription(
            String,
            'battery_charge',
            self.listener_callback,
            10)
    self.subscription  # prevent unused variable warning
      
    # Time interval in seconds
    timer_period = 5.0
    self.timer = self.create_timer(timer_period, self.get_battery_state)
     
    # Initialize battery level
    self.battery_voltage = 9.0 # Initialize the battery voltage level
    self.percent_charge_level = 1.0  # Initialize the percentage charge level
    self.decrement_factor = 0.98 # Used to reduce battery level each cycle
    self.perc_decrement_factor = 0.01
    self.charging = False
      
  def get_battery_state(self):
    """
    Callback function.
    This function gets called at the specific time interval.
    We decrement the battery charge level to simulate a real-world battery.
    """
    msg = BatteryState() # Create a message of this type 
    msg.voltage = self.battery_voltage 
    msg.percentage = self.percent_charge_level
      
    if self.charging:
      msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
      self.battery_voltage = 9.0
      self.percent_charge_level = self.percent_charge_level + (self.perc_decrement_factor * 20.0)
      if self.percent_charge_level >= 1.0:
        self.percent_charge_level = 1.0
        self.charging = False
    else:
      msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
      # Decrement the battery state 
      self.battery_voltage = self.battery_voltage * self.decrement_factor
      self.percent_charge_level = self.percent_charge_level - self.perc_decrement_factor
      if self.percent_charge_level < 0.0:
        self.percent_charge_level = 0.0
        
    self.publisher_battery_state.publish(msg) # Publish BatteryState message
    # self.get_logger().info('Battery percentage: "%f"' % msg.percentage)

  def listener_callback(self, msg):
    self.charging = True
    self.get_logger().info('Init charging')

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  battery_state_pub = BatteryStatePublisher()
  
  # Spin the node so the callback function is called.
  # Publish any pending messages to the topics.
  rclpy.spin(battery_state_pub)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  battery_state_pub.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
