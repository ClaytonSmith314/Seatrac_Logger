
import os
import time
from datetime import datetime
import toml
import rclpy
from rclpy.node import Node
from seatrac_interfaces.msg import ModemRec

CONFIG_FILE_PATH = "./seatrac_logger_config.toml"


def byte_list_to_int(bl):
    return (bl[0] << 24) | (bl[1] << 16) | (bl[2] << 8) | bl[3]

class SeatracLogger(Node):

    def __init__(self):
        super().__init__('logger')
        self.modem_subscriber_ = self.create_subscription(ModemRec, 'modem_rec', self.modem_callback, 10)

        if not os.path.exists("logger_data"):
            os.mkdir("logger_data")
        if not os.path.exists("logger_configurations"):
            os.mkdir("logger_configurations")

        time_str = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")

        #self.i=1

        with open(CONFIG_FILE_PATH) as config_file:
            config = toml.load(config_file)
            test_name = config["LoggerConfig"]["test_name"]

        with open(f"logger_configurations/{time_str}---{test_name}.toml", 'w') as config_save:
            with open(CONFIG_FILE_PATH) as config_file:
                config_save.write(config_file.read())

        self.output_file = open(f"logger_data/{time_str}---{test_name}.csv", 'w')
        self.output_file.write(
            "time, msg#, src_id, dest_id, local_flag, position_enhanced, position_flt_error, yaw, pitch, roll, local_depth, VOS, RSSI, usbl_rssi[0], usbl_rssi[1], usbl_rssi[2], usbl_rssi[3], range, azimuth, elevation, easting, northing, depth\n")

    def modem_callback(self, response):
        csv_line = (
            str(time.time())                            +", "+
            str(byte_list_to_int(response.packet_data)) +", "+
            str(response.msg_id)                        +", "+
            str(response.src_id)                        +", "+
            str(response.dest_id)                       +", "+
            str(response.local_flag)                    +", "+
            str(response.position_enhanced)             +", "+
            str(response.position_flt_error)            +", "+
            str(response.attitude_yaw)                  +", "+
            str(response.attitude_pitch)                +", "+
            str(response.attitude_roll)                 +", "+
            str(response.depth_local)                   +", "+
            str(response.vos)                           +", "+
            str(response.rssi)                          +", "+
            str(response.usbl_rssi[0])                  +", "+
            str(response.usbl_rssi[1])                  +", "+
            str(response.usbl_rssi[2])                  +", "+
            str(response.usbl_rssi[3])                  +", "+
            str(response.range_dist)                    +", "+
            str(response.usbl_azimuth)                  +", "+
            str(response.usbl_elevation)                +", "+
            str(response.usbl_fit_error)                +", "+
            str(response.position_easting)              +", "+
            str(response.position_northing)             +", "+
            str(response.position_depth)                +"\n"
        )
        self.output_file.write(csv_line)
        #self.i += 1


def main(args=None):
    rclpy.init(args=args)
    seatrac_logger = SeatracLogger()
    rclpy.spin(seatrac_logger)
    seatrac_logger.output_file.close()
    seatrac_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()