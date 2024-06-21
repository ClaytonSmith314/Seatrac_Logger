#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"

// Replace this with the serial port that your seatrac beacon is connected to.
#define SEATRAC_SERIAL_PORT "/dev/ttyUSB0"

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace narval::seatrac;

//TODO: maybe add commandline arg for serial port

// The class needs to inherit from both the ROS node and driver classes
class ModemRosNode : public rclcpp::Node, public SeatracDriver {
public:
  ModemRosNode()
      : Node("modem_ros_node"), SeatracDriver(SEATRAC_SERIAL_PORT), count_(0) {
    publisher_ =
        this->create_publisher<seatrac_interfaces::msg::ModemRec>("modem_send", 10);
    subscriber_ = 
        this->create_subscription<seatrac_interfaces::msg::ModemSend>("modem_rec", 10,
                      std::bind(&ModemRosNode::modem_send_callback, this, _1));
  }

private:

  rclcpp::Publisher<seatrac_interfaces::msg::ModemRec>::SharedPtr publisher_;
  rclcpp::Subscription<seatrac_interfaces::msg::ModemSend>::SharedPtr subscriber_;

  size_t count_;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModemRosNode>());
  rclcpp::shutdown();
  return 0;
}





