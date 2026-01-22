import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_= self.create_publisher(Twist,'cmd_vel',10)            
        timer_period = 0.01
        self.timer =self.create_timer(timer_period, self.timer_callback)
        self.i=0

        self.linear=0.0
        self.angular=0.0
        
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg):
        self.get_logger().info('I heard "%s"' % msg)
        self.linear=0.3*msg.axes[1]
        self.angular=0.3*msg.axes[0]
        
    
    def timer_callback(self):
        msg=Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher_.publish(msg)
        self.i+=1

    
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher=MinimalPublisher()
    rclpy.spin(minimal_publisher)

    minimal_publisher
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
