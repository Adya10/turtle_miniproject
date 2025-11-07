import rclpy
from rclpy.node import Node
from custom_nodes.srv import ControlTurtle
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class TurtleServiceServer(Node):

    def __init__(self):
        super().__init__('turtle_service_server')
        self.srv = self.create_service(ControlTurtle, 'control_turtle', self.control_turtle_callback)
        self.get_logger().info('Move server has been started')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.reset_service = self.create_client(Empty, '/reset')
        

    def control_turtle_callback(self, request, response):
        command = request.command.lower()
        twist = Twist()

        if command == 'start':
            twist.linear.x = 2.0
            twist.angular.z = 1.0
            self.publisher.publish(twist)
            response.success = True
            response.message = 'Turtle has started moving'

        elif command == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            response.success = True
            response.message = 'Turtle has stopped moving'

        elif command == 'reset':
            req = Empty.Request()
            self.reset_service.call_async(req)
            response.success = True
            response.message = 'Turtle has been reset to origin'

        else:
            response.success = False
            response.message = f'Invalid command {command}. Valid commands are "start", "stop" and "reset"'

        self.get_logger().info(f'Recieving command {command}')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurtleServiceServer()
    rclpy.spin(node)
    rclpy.shutdown()