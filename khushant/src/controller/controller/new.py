import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import DockStatus
from irobot_create_msgs.msg import HazardDetectionVector as detect
from irobot_create_msgs.msg import IrIntensityVector as ir
from nav_msgs.msg import Odometry 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import numpy as np
from sensor_msgs.msg import Imu
import math

obstacles = []
class move(Node):
    def __init__(self):
        super().__init__("Moving")
        self.move = self.create_publisher(Twist, "/robot_1/cmd_vel",1)
        self.timer = self.create_timer(.01, self.move_func)
        qos_profil = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.dock_sub = self.create_subscription(DockStatus, "/robot_1/dock_status",self.check_dock, qos_profile=qos_profil)
        self.dock_status = None
        self.haz_sub = self.create_subscription(detect, "/robot_1/hazard_detection", self.hazard_detect,qos_profile = qos_profil)
        self.hazard_status = None
        self.ir_sub = self.create_subscription(ir, "/robot_1/ir_intensity", self.ir_sense, qos_profile = qos_profil)
        self.ir_status = [0,0,0,0,0,0,0]
        #self.imu_sub = self.create_subscription(Imu, "/robot_1/imu", self.imu_sense, qos_profile = qos_profil)
        self.imu_status = None
        self.imu_sub = self.create_subscription(Odometry, "/robot_1/odom", self.imu_sense, qos_profile = qos_profil)
    
    def imu_sense(self, imu_msg:Odometry):
        self.get_logger().info("Getting Odometry")
        self.imu_status = [imu_msg.pose.pose.position[0], imu_msg.pose.pose.position[1]] 
    def ir_sense(self,ir_msg:ir):
        #self.get_logger().info("Value = ")
        self.ir_status[0] = ir_msg.readings[0].value
        self.ir_status[1] = ir_msg.readings[1].value
        self.ir_status[2] = ir_msg.readings[2].value
        self.ir_status[3] = ir_msg.readings[3].value
        self.ir_status[4] = ir_msg.readings[4].value
        self.ir_status[5] = ir_msg.readings[5].value
        self.ir_status[6] = ir_msg.readings[6].value
    def hazard_detect(self,haz_msg:detect):
        #self.get_logger().info("Checking for hazards")
        if len(haz_msg.detections)!=0:
            self.hazard_status = haz_msg.detections[0].type
    def check_dock(self,msg:DockStatus):
        #self.get_logger().info("Checking for docking")
        self.dock_status = msg.is_docked
   
    def move_func(self):
        msg = Twist()
        print(self.ir_status)
        if self.dock_status == False:
            msg.linear.x = .1
            self.move.publish(msg)
            if max(self.ir_status[0:3])<100:
                print("1")
                msg.linear.x = 0.0
                msg.angular.z = .2
                self.move.publish(msg)
                msg.linear.x = .1
                self.move.publish(msg)
                if self.ir_status[0] <10:
                    print("2")
                    msg.linear.x = 0.03
                    msg.angular.z = .4
                    self.move.publish(msg)
            elif max(self.ir_status) > 350 or self.hazard_status == 1:
                print("3")
                #self.get_logger().info("Moving the thing")
                msg.linear.x = 0.0
                #self.move.publish(msg) 
                min_index = self.ir_status.index(min(self.ir_status))
                #if min_index>=3:
                msg.angular.z = -.6
                #else:
                    #msg.angular.z = 0.8
                self.move.publish(msg)                   
                time.sleep(1)
                
def quaternion_to_euler(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Parameters:
    q (list or numpy array): A list or numpy array containing the quaternion in the format [w, x, y, z].
    
    Returns:
    numpy array: Euler angles in radians, in the format [roll, pitch, yaw].
    """
    # Extract components
    w, x, y, z = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([math.degrees(roll), math.degrees(pitch), math.degrees(yaw)])
def main(args = None):
    rclpy.init(args = args)
    node = move()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == 'main':
    main()
