
import time
import rclpy
from rclpy.node import Node
from seatrac_interfaces.msg import ModemSend, ModemRec

class SeatracLogger(Node):

    def __init__(self):
        super().__init__('logger')
        self.modem_publisher_  = self.create_publisher(ModemSend, 'modem_send', 10)
        self.modem_subscriber_ = self.create_subscription(ModemRec, 'modem_rec', self.modem_callback, 10)

        self.i = 0

        self.output_file = open("test_file.csv", 'w')
        self.output_file.write(
            "time, msg#, src_id, dest_id, local_flag, position_enhanced, position_flt_error, yaw, pitch, roll, local_depth, VOS, RSSI, usbl_rssi[0], usbl_rssi[1], usbl_rssi[2], usbl_rssi[3], range, azimuth, elevation, easting, northing, depth\n")
        

    def modem_callback(self, response):
        csv_line = (
            time.time()                 +", "+
            self.i                      +", "+
            response.msg_id             +", "+
            response.src_id             +", "+ #TODO: include source and dest ids in modem rec and add here
            response.dest_id            +", "+
            response.local_flag         +", "+
            response.position_enhanced  +", "+
            response.position_flt_error +", "+
            response.attitude_yaw       +", "+
            response.attitude_pitch     +", "+
            response.attitude_roll      +", "+
            response.depth_local        +", "+
            response.vos                +", "+
            response.rssi               +", "+
            response.usbl_rssi[0]       +", "+
            response.usbl_rssi[1]       +", "+
            response.usbl_rssi[2]       +", "+
            response.usbl_rssi[3]       +", "+
            response.range_dist         +", "+
            response.usbl_azimuth       +", "+
            response.usbl_elevation     +", "+
            response.usbl_fit_error     +", "+
            response.position_easting   +", "+
            response.position_northing  +", "+
            response.position_depth     +"\n"
        )
        self.output_file.write(csv_line)

        self.i += 1

        #TODO: add code to send out another message to the next beacon in the order
        

def main(args=None):
    rclpy.init(args=args)
    seatrac_logger = SeatracLogger()
    rclpy.spin(seatrac_logger)
    seatrac_logger.output_file.close()
    seatrac_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()