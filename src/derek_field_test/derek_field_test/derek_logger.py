
import os
from datetime import datetime
import rclpy
from rclpy.node import Node
from seatrac_interfaces.msg import ModemRec
from .seatrac_utils import CID_E

class SeatracLogger(Node):

    def __init__(self):
        super().__init__('logger')
        self.modem_subscriber_ = self.create_subscription(ModemRec, 'modem_rec', self.modem_callback, 10)

        if not os.path.exists("_derek_folder"):
            os.mkdir("_derek_folder")
        if not os.path.exists("_derek_folder/csv_data"):
            os.mkdir("_derek_folder/csv_data")

        time_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        self.output_file = open(f"_derek_folder/csv_data/{time_str}.csv", 'w')
        self.output_file.write(
            "timestamp, remote_beacon_id, position_flt_error, yaw, pitch, roll, local_depth, VOS, RSSI, usbl_rssi[0], usbl_rssi[1], usbl_rssi[2], usbl_rssi[3], range_time, range_dist, azimuth, elevation, easting, northing, remote_depth\n")

    def modem_callback(self, response):
        if(response.msg_id == CID_E.CID_PING_RESP):
            csv_line = (
                str(response.system_timestamp)    +", "+
                str(response.src_id)              +", "+
                str(response.position_flt_error)  +", "+
                str(response.attitude_yaw)        +", "+
                str(response.attitude_pitch)      +", "+
                str(response.attitude_roll)       +", "+
                str(response.depth_local)         +", "+
                str(response.vos)                 +", "+
                str(response.rssi)                +", "+
                str(response.usbl_rssi[0])        +", "+
                str(response.usbl_rssi[1])        +", "+
                str(response.usbl_rssi[2])        +", "+
                str(response.usbl_rssi[3])        +", "+
                str(response.range_time)          +", "+
                str(response.range_dist)          +", "+
                str(response.usbl_azimuth)        +", "+
                str(response.usbl_elevation)      +", "+
                str(response.usbl_fit_error)      +", "+
                str(response.position_easting)    +", "+
                str(response.position_northing)   +", "+
                str(response.position_depth)      +"\n"
            )
            self.output_file.write(csv_line)


def main(args=None):
    rclpy.init(args=args)
    seatrac_logger = SeatracLogger()
    rclpy.spin(seatrac_logger)
    seatrac_logger.output_file.close()
    seatrac_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()