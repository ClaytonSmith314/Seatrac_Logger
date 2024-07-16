
import time
import rclpy
from rclpy.node import Node
from seatrac_interfaces.msg import ModemSend, ModemRec
import toml
from .seatrac_utils import CID_E, AMSGTYPE_E

CONFIG_FILE_PATH = "./seatrac_logger_config.toml"
TIMER_PERIOD_SECONDS = 0.01


class SeatracPinger(Node):

    def __init__(self):
        super().__init__('pinger')

        with open(CONFIG_FILE_PATH) as config_file:
            logger_config = toml.load(config_file)["DerekTest"]

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

        self.beacon_id_list_index = 0
        self.rounds_of_pings_sent = 0

        self.send_ping()


    def send_ping(self):
        request = ModemSend()
        request.msg_id      = CID_E.CID_PING_SEND
        request.dest_id     = self.other_beacon_ids[self.beacon_id_list_index]
        request.msg_type    = self.msg_type

        self.modem_publisher_.publish(request)

        self.beacon_id_list_index += 1
        if self.beacon_id_list_index >= len(self.other_beacon_ids):
            self.beacon_id_list_index = 0
            self.rounds_of_pings_sent += 1
            if self.rounds_of_pings_sent > self.num_rounds_of_pings:
                pass # TODO: kill the node here

    def modem_callback(self, response):
        if response.msg_id == CID_E.CID_PING_RESP:
            self.send_ping()
        if response.msg_id == CID_E.CID_PING_ERROR:
            self.send_ping()
            self.get_logger().error(f"Seatrac Ping Error. CST_E Error Code: {str(response.command_status_code)}")


def main(args=None):
    rclpy.init(args=args)
    seatrac_logger = SeatracPinger()
    rclpy.spin(seatrac_logger)
    rclpy.shutdown()


if __name__ == '__main__':
    main()