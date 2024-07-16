
import time
import rclpy
from rclpy.node import Node
from seatrac_interfaces.msg import ModemSend, ModemRec
import toml
from .seatrac_utils import CID_E, AMSGTYPE_E

CONFIG_FILE_PATH = "./seatrac_logger_config.toml"
TIMER_PERIOD_SECONDS = 0.01


def int_to_bytes_list(num):
    bytes = [0]*30
    bytes[0] = (num >> 24) & 0xFF
    bytes[1] = (num >> 16) & 0xFF
    bytes[2] = (num >> 8) & 0xFF
    bytes[3] = num & 0xFF
    return bytes

class SeatracPinger(Node):

    def __init__(self):
        super().__init__('pinger')

        with open(CONFIG_FILE_PATH) as config_file:
            logger_config = toml.load(config_file)["LoggerConfig"]

            self.num_rounds_of_pings = logger_config["num_pings_per_beacon"]
            self.other_beacon_ids    = logger_config["other_beacon_ids"]
            self.use_advanced_usbl   = logger_config["use_advanced_usbl"]
            self.timeout             = logger_config["wait_for_response_timeout_seconds"]

        for beacon_id in self.other_beacon_ids:
            assert beacon_id>=1 and beacon_id<=15

        if self.use_advanced_usbl:
            self.msg_type = AMSGTYPE_E.MSG_REQX
        else:
            self.msg_type = AMSGTYPE_E.MSG_REQU

        self.modem_publisher_  = self.create_publisher(ModemSend, 'modem_send', 10)
        self.modem_subscriber_ = self.create_subscription(ModemRec, 'modem_rec', self.modem_callback, 10)

        self.timer = self.create_timer(TIMER_PERIOD_SECONDS, self.timer_callback)
        self.time_since_pinged = 0.0
        self.received_response = False

        self.beacon_id_list_index = 0
        self.rounds_of_pings_sent = 0

        self.send_ping()


    def send_ping(self):
        request = ModemSend()
        request.msg_id      = CID_E.CID_ECHO_SEND
        request.dest_id     = self.other_beacon_ids[self.beacon_id_list_index]
        request.msg_type    = self.msg_type
        request.packet_len  = 4
        request.packet_data = int_to_bytes_list(self.rounds_of_pings_sent)

        self.modem_publisher_.publish(request)

        self.beacon_id_list_index += 1
        if self.beacon_id_list_index >= len(self.other_beacon_ids):
            self.beacon_id_list_index = 0
            self.rounds_of_pings_sent += 1
            if self.rounds_of_pings_sent > self.num_rounds_of_pings:
                pass # TODO: kill the node here

    def modem_callback(self, response):
        self.received_response = True
    
    def timer_callback(self):
        if self.received_response or self.time_since_pinged>self.timeout:
            self.time_since_pinged = 0.0
            self.received_response = False
            self.send_ping()
        else:
            self.time_since_pinged += TIMER_PERIOD_SECONDS


def main(args=None):
    rclpy.init(args=args)
    seatrac_logger = SeatracPinger()
    rclpy.spin(seatrac_logger)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
