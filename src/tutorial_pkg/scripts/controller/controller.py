#!/usr/bin/env python3 
 
import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class
from sensor_msgs.msg import BatteryState # Enable use of the sensor_msgs/BatteryState message type
from std_msgs.msg import String, Bool
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import time
  
class Controller(Node):
  def __init__(self):
    super().__init__('controller')
    self.charge_pub = self.create_publisher(String, 'battery_charge', 10)
    self.explore_pub = self.create_publisher(Bool, 'explore/resume', 10)
    self.action_pub = self.create_publisher(Bool, 'action', 10)
    self.battery_sub = self.create_subscription(
            BatteryState,
            'battery_status',
            self.battery_callback,
            10)
    self.battery_sub  # prevent unused variable warning
    self.waiting_to_charge = False
    self.nav = BasicNavigator()
    
  def publish_action(self, data: str):
      action = String()
      action.data = data
      self.action_pub.publish(action)

  def battery_callback(self, msg: BatteryState):
    if(msg.percentage < 0.30 and msg._power_supply_status != BatteryState.POWER_SUPPLY_STATUS_CHARGING):
      self.publish_action("Stop exploration")
      explore_msg = Bool()
      explore_msg.data = False
      self.explore_pub.publish(explore_msg)
      time.sleep(2)
      goal_pose = PoseStamped()
      goal_pose.header.frame_id = 'map'
      goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
      goal_pose.pose.position.x = 0.0
      goal_pose.pose.position.y = 0.0
      self.publish_action("Return to base")
      self.nav.goToPose(goal_pose)
      while not self.nav.isTaskComplete():
        time.sleep(1)
      self.publish_action("Start charging")
      self.charge_pub.publish(String())
      self.waiting_to_charge = True
    elif(self.waiting_to_charge and msg.power_supply_status != BatteryState.POWER_SUPPLY_STATUS_CHARGING):
      self.publish_action("Restart exploration")
      explore_msg = Bool()
      explore_msg.data = True
      self.explore_pub.publish(explore_msg)
      self.waiting_to_charge = False

def main(args=None):
  rclpy.init(args=args)
  controller = Controller()
  rclpy.spin(controller)
  controller.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
