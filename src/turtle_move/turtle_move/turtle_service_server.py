import rclpy
from rclpy.node import Node
from custom_nodes.srv import ControlTurtle

class TurtleServiceServer(Node):

    def __init__(self):
        super().__init__('turtle_service_server')
        self.srv = self.create_service(ControlTurtle, 'control_turtle', self.control_turtle_callback)
        self.get_logger().info('Move server has been started')

        self.turtle_running = False

    def control_turtle_callback(self, request, response):
        command = request.command.lower()

        if command == 'start':
            self.turtle_running = True
            response.success = True
            response.message = 'Turtle has started moving'

        elif command == 'stop':
            self.turtle_running = False
            response.success = True
            response.message = 'Turtle has stopped moving'

        elif command == 'reset':
            self.turtle_running = False
            response.success = True
            response.message = 'Turtle has been reset to initial position'

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