
import rclpy
from rclpy.node import Node
from seatrac_interfaces.msg import ModemSend, ModemRec
from .seatrac_utils import CID_E, NAV_QUERY_E, CST_E

CONFIG_FILE_PATH = "./seatrac_logger_config.toml"
TIMER_PERIOD_SECONDS = 0.01


class SeatracPinger(Node):

    def __init__(self):
        super().__init__('pinger')
            
        self.declare_parameter("beacon_ids_to_ping", [15])

        self.other_beacon_ids = self.get_parameter("beacon_ids_to_ping").get_parameter_value().integer_array_value

        for beacon_id in self.other_beacon_ids:
            assert beacon_id>=1 and beacon_id<=15

        self.modem_publisher_  = self.create_publisher(ModemSend, 'modem_send', 10)
        self.modem_subscriber_ = self.create_subscription(ModemRec, 'modem_rec', self.modem_callback, 10)

        self.beacon_id_list_index = 0
        self.rounds_of_pings_sent = 0

        self.first_response = True

    def send_ping(self):
        request = ModemSend()
        request.msg_id          = CID_E.CID_NAV_QUERY_SEND
        request.dest_id         = self.other_beacon_ids[self.beacon_id_list_index]
        request.nav_query_flags = NAV_QUERY_E.QRY_ATTITUDE | NAV_QUERY_E.QRY_DEPTH | NAV_QUERY_E.QRY_TEMP
        
        self.modem_publisher_.publish(request)

        self.beacon_id_list_index += 1
        if self.beacon_id_list_index >= len(self.other_beacon_ids):
            self.beacon_id_list_index = 0
            self.rounds_of_pings_sent += 1

    def modem_callback(self, response):
        if response.msg_id == CID_E.CID_NAV_QUERY_RESP:
            self.send_ping()
        if response.msg_id == CID_E.CID_NAV_ERROR:
            self.send_ping()
            self.get_logger().error(f"Seatrac Ping Error. Target Beacon Id: {response.target_id}. Error Code: {CST_E.to_str(response.command_status_code)}")
        if response.msg_id == CID_E.CID_NAV_QUERY_SEND and not response.command_status_code == CST_E.CST_OK:
            self.send_ping()
            self.get_logger().error(f"Seatrac Ping Send Error. Target Beacon Id: {response.target_id}. Error Code: {CST_E.to_str(response.command_status_code)}")
        if self.first_response:
            self.get_logger().info("got first response")
            self.first_response = False
            self.send_ping()

def main(args=None):
    rclpy.init(args=args)
    seatrac_logger = SeatracPinger()
    rclpy.spin(seatrac_logger)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
