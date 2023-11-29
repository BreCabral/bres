import rclpy
from rclpy.node import Node
import RPi.GPIO as gpio

from std_msgs.msg import String

gpio.setmode(gpio.BCM)
gpio.setup(20, gpio.OUT)
gpio.setup(21, gpio.IN, pull_up_down = gpio.PUD_DOWN)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.aux = True
        self.valor = 0

    def timer_callback(self):
        if self.aux == True:
            gpio.output(20, gpio.HIGH)
            self.aux = False
        else:
            gpio.output(20, gpio.LOW)
            self.aux = True
        self.valor = gpio.input(21)
        msg = String()
        msg.data = 'Hello World: %d ' % self.valor
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    gpio.cleanup()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()