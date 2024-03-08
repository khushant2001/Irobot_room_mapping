import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import Dock

class dock(Node):
    def __init__(self):
        super().__init__("Check_dock")
        self.sub = self.create_subscription(Dock, "/robot_3/dock", self.check,10)
    
    def check(self):
        msg = Dock()
        msg.is_docked = False
        self.pub.publish(msg)
def main(args = None):
    rclpy.init(args = args)
    node = dock(Node)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == 'main':
    main()
