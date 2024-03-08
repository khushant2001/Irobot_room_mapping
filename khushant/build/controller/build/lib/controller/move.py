import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import DockStatus
from irobot_create_msgs.msg import HazardDetectionVector as detect
from irobot_create_msgs.msg import IrIntensityVector as ir
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time

class move(Node):
    def __init__(self):
        super().__init__("Moving")
        self.move = self.create_publisher(Twist, "/robot_1/cmd_vel",10)
        self.timer = self.create_timer(.5, self.move_func)
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
        self.get_logger().info("Checking for hazards")
        if len(haz_msg.detections)!=0:
            self.hazard_status = haz_msg.detections[0].type
    def check_dock(self,msg:DockStatus):
        self.get_logger().info("Checking for docking")
        self.dock_status = msg.is_docked
        
    def move_func(self):
        msg = Twist()
        print(self.ir_status)
        if self.dock_status == False:
            msg.linear.x = 1.0
            self.move.publish(msg)
            if max(self.ir_status) > 300:
                #self.get_logger().info("Moving the thing")
                msg.linear.x = 0.0 
                min_index = self.ir_status.index(min(self.ir_status))
                if min_index>3:
                    msg.angular.z = -.5
                elif min_index<3:
                    msg.angular.z = .5
                else:
                    msg.linear.x = -10.0
                self.move.publish(msg)                   
                #time.sleep(2)
                
def main(args = None):
    rclpy.init(args = args)
    node = move()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == 'main':
    main()

