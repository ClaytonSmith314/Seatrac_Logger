#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/commands.h>
#include <seatrac_driver/messages/Messages.h>

#include "rclcpp/rclcpp.hpp"
#include "seatrac_interfaces/msg/modem_rec.hpp"
#include "seatrac_interfaces/msg/modem_send.hpp"

#include "toml.hpp"

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
    RCLCPP_INFO(this->get_logger(), "Starting seatrac modem Node");
    publisher_ =
        this->create_publisher<seatrac_interfaces::msg::ModemRec>("modem_rec", 10);
    subscriber_ = 
        this->create_subscription<seatrac_interfaces::msg::ModemSend>("modem_send", 10,
                      std::bind(&ModemRosNode::modem_send_callback, this, _1));

    //upload_config_settings(&this);

  }

  // this method is called on any message returned by the beacon.
  // it copies the modem data to a ros message of type ModemRec
  void on_message(CID_E msgId, const std::vector<uint8_t> &data) {
    switch (msgId) {
      default: {
        RCLCPP_INFO(this->get_logger(), "Received unknown message from seatrac modem. msgId: %d", msgId); 
      } break;
      case CID_DAT_RECEIVE: {
        messages::DataReceive response;     //struct that contains response fields
        response = data;                    //operator overload fills in response struct with correct data

        auto msg = seatrac_interfaces::msg::ModemRec();
        msg.msg_id = CID_DAT_RECEIVE;
        msg.packet_len = response.packetLen;
        msg.local_flag = response.localFlag;
        std::memcpy(&msg.packet_data, response.packetData, response.packetLen);
        cpyFixtoRosmsg(msg, response.acoFix);

        RCLCPP_INFO(this->get_logger(), "Publishing ModemRec CID_DAT_RECEIVE");
        publisher_->publish(msg);
      } break;
      case CID_DAT_ERROR: {
        messages::DataError response;
        response = data;
        std::ostringstream err;
        err << "Error with seatrac modem data message." << std::endl << response;
        RCLCPP_ERROR(this->get_logger(), err.str().c_str());
      } break;

      case CID_PING_RESP: {
        messages::PingResp response;
        response = data;

        auto msg = seatrac_interfaces::msg::ModemRec();
        msg.msg_id = CID_PING_RESP;
        msg.packet_len = 0;
        msg.local_flag = true; //Ping messages are not sniffed.
        cpyFixtoRosmsg(msg, response.acoFix);

        RCLCPP_INFO(this->get_logger(), "Publishing ModemRec CID_PING_RESP");
        publisher_->publish(msg);
      } break;
      case CID_PING_ERROR: {
        messages::PingError response;
        response = data;
        std::ostringstream err;
        err << "Error with seatrac modem ping message." << std::endl << response;
        RCLCPP_ERROR(this->get_logger(), err.str().c_str()); //TODO: add response diagnostic data to message
      } break;

      case CID_STATUS:
        //Too many status messages so bypasing display
        break;
    }
  }

private:

  rclcpp::Publisher<seatrac_interfaces::msg::ModemRec>::SharedPtr publisher_;
  rclcpp::Subscription<seatrac_interfaces::msg::ModemSend>::SharedPtr subscriber_;

  size_t count_;

  // recieves command to modem from the ModemRec topic and sends the command
  // to the modem
  void modem_send_callback(const seatrac_interfaces::msg::ModemSend::SharedPtr rosmsg) {
    CID_E msgId = static_cast<CID_E>(rosmsg->msg_id);
    switch(msgId) {
      default: {
        RCLCPP_ERROR(this->get_logger(), "Unsupported seatrac message id for broadcasting messages: %d", msgId);
      } break;
      case CID_DAT_SEND: {
        messages::DataSend::Request req; //struct contains message to send to modem

        req.destId    = static_cast<BID_E>(rosmsg->dest_id);
        req.msgType   = static_cast<AMSGTYPE_E>(rosmsg->msg_type);
        req.packetLen = std::min(rosmsg->packet_len, (uint8_t)sizeof(req.packetData));
        
        std::memcpy(req.packetData, rosmsg->packet_data.data(), req.packetLen);
        RCLCPP_INFO(this->get_logger(), "Seatrac modem broadcasting CID_DAT_SEND message. String is '%s'", req.packetData);
        this->send(sizeof(req), (const uint8_t*)&req);

      } break;

      case CID_PING_SEND: {
        messages::PingSend::Request req;
        req.target    = static_cast<BID_E>(rosmsg->dest_id);
        req.pingType  = static_cast<AMSGTYPE_E>(rosmsg->msg_type);
        RCLCPP_INFO(this->get_logger(), "Seatrac modem broadcasting CID_DAT_SEND message");
        this->send(sizeof(req), (const uint8_t*)&req);
      } break;
      //TODO: add case for calibration
    }
  }

  //copies the fields from the acofix struct into the ModemRec ros message
  inline void cpyFixtoRosmsg(seatrac_interfaces::msg::ModemRec& msg, ACOFIX_T& acoFix) {
    msg.attitude_yaw = acoFix.attitudeYaw;
    msg.attitude_pitch = acoFix.attitudePitch;
    msg.attitude_roll = acoFix.attitudeRoll;
    msg.depth_local = acoFix.depthLocal;
    msg.vos = acoFix.vos;
    msg.rssi = acoFix.rssi;


    msg.range_valid = (acoFix.flags & RANGE_VALID)? true:false;
    msg.usbl_valid = (acoFix.flags & USBL_VALID)? true:false;
    msg.position_valid = (acoFix.flags & POSITION_VALID)? true:false;

    msg.position_enhanced = (acoFix.flags & POSITION_ENHANCED)? true:false;
    msg.position_flt_error = (acoFix.flags & POSITION_FLT_ERROR)? true:false;

    if(msg.range_valid) {
      msg.range_count = acoFix.range.count;
      msg.range_time = acoFix.range.time;
      msg.range_dist = acoFix.range.dist;
    }
    if(msg.usbl_valid) {
      msg.usbl_channels = acoFix.usbl.channelCount;
      std::memcpy(&msg.usbl_rssi, acoFix.usbl.rssi, acoFix.usbl.channelCount);
      msg.usbl_azimuth = acoFix.usbl.azimuth;
      msg.usbl_elevation = acoFix.usbl.elevation;
      msg.usbl_fit_error = acoFix.usbl.fitError;
    }
    if(msg.position_valid) {
      msg.position_easting = acoFix.position.easting;
      msg.position_northing = acoFix.position.northing;
      msg.position_depth = acoFix.position.depth;
    }
  }
};

toml::ex::parse_result upload_config_settings(SeatracDriver& seatrac, std::string config_file_path="./seatrac_logger_config.toml") {

    std::cout << "starting upload settings to seatrac beacon" << std::endl;

    auto config = toml::parse_file(config_file_path);
    auto seatrac_config = config["SeatracConfig"];

    SETTINGS_T settings = command::settings_get(seatrac).settings;
    std::cout << "retrieved current beacon settings. Old settings: " << std::endl
              << settings << std::endl;

    switch((int)(seatrac_config["status_report_frequency_hertz"].value_or(0.0)*10)) {
        case 0:   settings.statusFlags = STATUS_MODE_MANUAL; break;
        case 10:  settings.statusFlags = STATUS_MODE_1HZ;    break;
        case 25:  settings.statusFlags = STATUS_MODE_2HZ5;   break;
        case 50:  settings.statusFlags = STATUS_MODE_5HZ;    break;
        case 100: settings.statusFlags = STATUS_MODE_10HZ;   break;
        case 250: settings.statusFlags = STATUS_MODE_25HZ;   break;
        default:  std::cout << "Seatrac Config Error: value of status_report_frequency_hertz is invalid" << std::endl;
    };
    settings.status_output = (STATUS_BITS_E)(
          ENVIRONMENT    * seatrac_config["status_include_temp_pressure_depth_vos"].value_or(false)
        | ATTITUDE       * seatrac_config["status_include_yaw_pitch_roll"].value_or(false)
        | MAG_CAL        * seatrac_config["status_include_mag_cal_data"].value_or(false)
        | ACC_CAL        * seatrac_config["status_include_accel_cal_data"].value_or(false)
        | AHRS_RAW_DATA  * seatrac_config["status_include_uncompensated_accel_mag_gyro"].value_or(false)
        | AHRS_COMP_DATA * seatrac_config["status_include_compensated_accel_mag_gyro"].value_or(false)
    );

    settings.envFlags = (ENV_FLAGS_E)(
          AUTO_VOS          * seatrac_config["auto_calc_velocity_of_sound"].value_or(true)
        | AUTO_PRESSURE_OFS * seatrac_config["auto_calc_pressure_offset"].value_or(true)
    );
    // settings.envPressureOfs = 0; //value will be overwritten if auto_calc_pressure_offset is true
    // settings.envVos = 0; //value will be overwritten if auto_calc_velocity_of_sound is true

    settings.envSalinity = (uint8_t)(10*seatrac_config["env_salinity_ppt"].value_or(0.0));

    settings.ahrsFlags = (AHRS_FLAGS_E)seatrac_config["automatic_mag_calibration"].value_or(false);
    
    settings.ahrsYawOfs   = 0;
    settings.ahrsPitchOfs = 0;
    settings.ahrsRollOfs  = 0;

    XCVR_TXMSGCTRL_E msgctrl = 
    seatrac_config["transceiver_block_send_all"].value_or(false)? 
        XCVR_TXMSG_BLOCK_ALL
        : (seatrac_config["transceiver_block_send_response"].value_or(false)? 
            XCVR_TXMSG_BLOCK_RESP
            : XCVR_TXMSG_ALLOW_ALL);

    settings.xcvrFlags = (XCVR_FLAGS_E)(
          USBL_USE_AHRS      //* seatrac_config["usbl_use_AHRS"].value_or(true)
        | XCVR_POSFLT_ENABLE * seatrac_config["position_filter_enabled"].value_or(true)
        | XCVR_USBL_MSGS     * seatrac_config["report_transceiver_usbl_msgs"].value_or(false)
        | XCVR_FIX_MSGS      * seatrac_config["report_transceiver_fix_msgs"].value_or(false)
        | XCVR_DIAG_MSGS     * seatrac_config["report_transceiver_msg_diagnostics"].value_or(false)
        | (msgctrl << 3) //XCVR_TXMSGCTRL_E uses the 3rd and 4th bits of the transciever flags
    );

    settings.xcvrBeaconId = (BID_E)seatrac_config["beacon_id"].value_or(0);
    if(settings.xcvrBeaconId<1 || settings.xcvrBeaconId>15)
        throw; //"Seatrac Config Error: beacon_id must be between 0 and 15";

    settings.xcvrRangeTmo = (uint16_t)seatrac_config["transceiver_range_timeout_meters"].value_or(1000);
    if(settings.xcvrRangeTmo>3000 || settings.xcvrRangeTmo<1000)
        std::cout << "Seatrac Config Error: transceiver_range_timeout_meters must be between 1000 and 3000 m" << std::endl;
    settings.xcvrRespTime = (uint16_t)seatrac_config["transceiver_response_delay_milliseconds"].value_or(10);
    if(settings.xcvrRespTime>1000 || settings.xcvrRespTime<10)
        std::cout << "Seatrac Config Error: transceiver_response_delay_milliseconds must be between 10 and 1000 ms" << std::endl;

    settings.xcvrPosfltVel = (uint8_t)(seatrac_config["pos_filter_velocity_limit_meters_per_sec"].value_or(3));
    settings.xcvrPosfltAng = (uint8_t)(seatrac_config["pos_filter_angle_limit_degrees"].value_or(10));
    settings.xcvrPosfltTmo = (uint8_t)(seatrac_config["pos_filter_timeout_seconds"].value_or(60));

    std::cout << settings << std::endl;

    std::cout << "uploading settings to beacon";
    messages::SettingsSet resp = command::settings_set(seatrac, settings);
    if(resp.statusCode != CST_OK)
        throw "Error saving settings to seatrac beacon";

    return config;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto modem_ros_node = std::make_shared<ModemRosNode>();
  upload_config_settings(*modem_ros_node.get());
  rclcpp::spin(modem_ros_node);
  rclcpp::shutdown();
  return 0;
}





