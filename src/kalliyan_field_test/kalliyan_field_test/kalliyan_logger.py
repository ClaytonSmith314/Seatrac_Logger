
import os
import time
from datetime import datetime
import toml
import rclpy
from rclpy.node import Node
from seatrac_interfaces.msg import ModemRec
from .seatrac_utils import CID_E

CONFIG_FILE_PATH = "./seatrac_logger_config.toml"


def byte_list_to_int(bl):
    return (bl[0] << 24) | (bl[1] << 16) | (bl[2] << 8) | bl[3]

class SeatracLogger(Node):

    def __init__(self):
        super().__init__('logger')
        self.modem_subscriber_ = self.create_subscription(ModemRec, 'modem_rec', self.modem_callback, 10)

        if not os.path.exists("kalliyan_logger_data"):
            os.mkdir("kalliyan_logger_data")

        time_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        test_name = "KalliyanTest"

        self.output_file = open(f"kalliyan_logger_data/{time_str}.csv", 'w')
        self.output_file.write(
            "timestamp, remote_beacon_id, position_flt_error, yaw, pitch, roll, local_depth, VOS, RSSI, range_time, range_dist, azimuth, elevation, easting, northing, remote_depth, remote_yaw, remote_pitch, remote_roll, remote_temp\n")

    def modem_callback(self, response):
        if(response.msg_id == CID_E.CID_NAV_QUERY_RESP):
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
                # str(response.usbl_rssi[0])        +", "+
                # str(response.usbl_rssi[1])        +", "+
                # str(response.usbl_rssi[2])        +", "+
                # str(response.usbl_rssi[3])        +", "+
                str(response.range_time)          +", "+
                str(response.range_dist)          +", "+
                str(response.usbl_azimuth)        +", "+
                str(response.usbl_elevation)      +", "+
                str(response.usbl_fit_error)      +", "+
                str(response.position_easting)    +", "+
                str(response.position_northing)   +", "+
                str(response.remote_depth)        +", "+
                str(response.remote_yaw)          +", "+
                str(response.remote_pitch)        +", "+
                str(response.remote_roll)         +", "+
                str(response.remote_temp)         +"\n"
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