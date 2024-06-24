
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SeatracLogger(Node):

    def __init__(self):
        super().__init__('logger')
        self.modem_publisher_  = self.create_publisher(ModemSend, 'modem_send', 10)
        self.modem_subscriber_ = self.create_subscriber(ModemRec, 'modem_rec', self.modem_callback)

        self.i = 0

        self.output_file = open("test_file.csv", 'w')
        self.output_file.write("I'm writing a thing")

    def modem_callback(self, response):
        self.output_file.write("I'm writing a line!!")

        

def main(args=None):
    rclpy.init(args=args)

    seatrac_logger = SeatracLogger()

    rclpy.spin(seatrac_logger)
    seatrac_logger.output_file.close()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    seatrac_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()