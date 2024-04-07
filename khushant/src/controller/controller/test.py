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

obstacles = np.array([
    [0]
])
i = 0
error = [0]
integral = [0]
time_step = .001
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
        self.initial_position = np.array([[0,0]])
        self.dock_sub = self.create_subscription(DockStatus, "/robot_1/dock_status",self.check_dock, qos_profile=qos_profil)
        self.dock_status = None
        self.haz_sub = self.create_subscription(detect, "/robot_1/hazard_detection", self.hazard_detect,qos_profile = qos_profil)
        self.hazard_status = None
        self.ir_sub = self.create_subscription(ir, "/robot_1/ir_intensity", self.ir_sense, qos_profile = qos_profil)
        self.ir_status = [0,0,0,0,0,0,0]
        #self.imu_sub = self.create_subscription(Imu, "/robot_1/imu", self.imu_sense, qos_profile = qos_profil)
        self.imu_status = np.array([[0,0]])
        self.imu_sub = self.create_subscription(Odometry, "/robot_1/odom", self.imu_sense, qos_profile = qos_profil)
    
    def imu_sense(self, imu_msg:Odometry):
        global i
        #self.get_logger().info("Getting Odometry")
        self.imu_status = np.array([
            [math.floor(5*imu_msg.pose.pose.position._x), math.floor(5*imu_msg.pose.pose.position._y)]]) 
        if i == 0:
            self.initial_position[0,0] = self.imu_status[0,0]
            self.initial_position[0,1] = self.imu_status[0,1]
            i = i + 1
        obstacle_detection(self.imu_status - self.initial_position)

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
        initial = True
        tolerance = np.array([
            [5]
        ])
        if initial:
            setpoint = 70
            error = setpoint - self.ir_status[0]
            if (self.ir_status[3]) > 20:
                msg.linear.x = .01
                msg.angular.z = -0.7
            else:
                msg.angular.z = actuation(error)
                msg.linear.x = 1.0
            #print(msg.angular.z)
            self.move.publish(msg)

            if (self.initial_position[0,0] - self.imu_status[0,1]) < tolerance:
                initial = False

ki = 0.05
kp = .027
kd = 0.001
def actuation(e):
    error.append(e)
    integral.append(error[-1]*time_step + integral[-1])
    derivative = (error[-1] - error[-2]) / time_step if len(error) > 1 else 0
    control = kp * error[-1] + ki * integral[-1] + kd * derivative
    #print(-control)
    return (control)

def obstacle_detection(points):
    points[0,0] = abs(points[0][0])
    points[0,1] = abs(points[0][1])
    #print(points[0,0])
    #print(points[0,1])
    global obstacles
    o_shape = obstacles.shape
    
    if points[0,0] > o_shape[0]-1:
        new_column = np.ones((points[0,0],o_shape[1]))
        
        obstacles = np.vstack([obstacles, new_column])

    elif points[0,1] > o_shape[1]-1:  
        new_row = np.zeros((o_shape[0],1))
        new_row[0] = 1
        obstacles = np.hstack([obstacles, new_row])
    else:
        print("h")
        obstacles[points[0,0]][points[0,1]] = 1
    print("New")
    print(obstacles)

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
