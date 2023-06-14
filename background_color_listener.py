import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from turtlesim.msg import Color


class ColorChangeSubscriber(Node):

    def __init__(self):
        super().__init__("color_changes_subscriber")
        self.subscription = self.create_subscription(Color, "/turtlesim_background", self.listener_callback, 10)
        self.client=self.create_client(SetParameters, '/turtlesim/set_parameters')

    def listener_callback(self, msg):
        print(msg.r)
        print("found new color")
        req = SetParameters.Request()
        req.parameters = [
            Parameter('background_r', value=msg.r).to_parameter_msg(),
            Parameter('background_g', value=msg.g).to_parameter_msg(),
            Parameter('background_b', value=msg.b).to_parameter_msg()
        ]
        future = self.client.call_async(req)
        #rclpy.spin_until_future_complete(self, future)
        ## Process the response
        #if future.result() is not None:
        #    self.get_logger().info(f'Result: {future.result().success}')
        #else:
        #    self.get_logger().info('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    subscriber = ColorChangeSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
