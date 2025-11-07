import rclpy
from rclpy.node import Node
from custom_nodes.srv import ControlTurtle

class TurtleServiceClient(Node):

    def __init__(self):
        super().__init__('turtle_service_client')
        self.client = self.create_client(ControlTurtle, 'control_turtle')

        while not self.client.wait_for_service(1.0):
            self.get_logger().warn('Waiting for server to be available...')

        self.get_logger().info('Service client has been started')

    def send_command(self, command: str):

        request = ControlTurtle.Request()

        request.command = command

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Response: success = {response.success}, message = {response.message}')
        else:
            self.get_logger().error('Call service failed')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleServiceClient()

    try:
        while True:
            cmd = input('\nEnter a command (start / stop / reset / quit)')
            if cmd == 'quit':
                break
            elif cmd in ['start', 'stop', 'reset']:
                node.send_command(cmd)
            else:
                print('invalid command please try again')
    
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
