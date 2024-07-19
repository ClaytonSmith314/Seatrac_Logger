#include <iostream>

#include <seatrac_driver/SeatracDriver.h>
#include <seatrac_driver/messages/Messages.h>
#include <seatrac_driver/commands.h>
#include <seatrac_driver/Calibration.h>

using namespace std::chrono_literals;
using namespace narval::seatrac;

class MyDriver : public SeatracDriver
{
    public:

    MyDriver(const std::string& serialPort = "/dev/ttyUSB0") :
        SeatracDriver(serialPort)
    {}

    // this method is called on any message returned by the beacon.
    void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
        switch(msgId) {
            default:
                //std::cout << "Got message : " << msgId << std::endl << std::flush;
                break;

            case CID_STATUS: {
                messages::Status status;
                status = data;
                calibration::printCalFeedback(std::cout, status);
            } break;
        }
    }
};

bool yn_answer() {
    while(true) {
        char yorn;
        if(scanf("%c", &yorn)) {
            if(yorn == 'y') {std::cin.get(); return true;}
            if(yorn == 'n') {std::cin.get(); return false;}
        }

        std::cout << "Invalid response. Please enter 'y' or 'n': ";
    }
}

int main(int argc, char *argv[]) {

    std::cout << "=== Seatrac Beacon Setup Tool ==="    << std::endl << std::endl;
            //   << "Procedure:"                           << std::endl
            //   << " - Connect to Beacon"                 << std::endl
            //   << " - Set Beacon Id"                     << std::endl
            //   << " - Set Beacon Settings"               << std::endl
            //   << " - Calibrate Magnetometer"            << std::endl;

    bool cont = true;
    while(cont) {
        std::cout << "Enter Serial Port (or blank for default '/dev/ttyUSB0'): ";
        char serial_port[20];
        fgets(serial_port, sizeof(serial_port), stdin);
        serial_port[strlen(serial_port)-1] = 0x00;
        if(strlen(serial_port) == 0) strcpy(serial_port, "/dev/ttyUSB0");

        {
        std::cout << "Connecting to Beacon... ";
        MyDriver seatrac(serial_port);
        command::status_config_set(seatrac, (STATUS_BITS_E)0x0); 
        std::cout << "Done" << std::endl;

        std::cout << "Retrieving Current Settings... ";
        SETTINGS_T settings = command::settings_get(seatrac).settings;
        std::cout << "Done" << std::endl;

        // Change beacon id
        std::cout << "Current Beacon Id: " << settings.xcvrBeaconId << std::endl
                  << "Change Beacon Id (y/n)? ";
        if(yn_answer()) {
            int bid;
            while(true) {
                std::cout << "Enter New Beacon Id (integer between 1 and 15 inclusive): ";
                if(scanf("%d", &bid) && bid<=15 && bid>=1) break;
                while(std::cin.get()!='\n');
                std::cout << "Invalid Beacon Id. Id should be an integer between 1 and 15 inclusive." << std::endl;
            }
            std::cin.get();
            std::cout << "Setting Beacon Id to " << bid << "... ";
            command::set_beacon_id(seatrac, (BID_E)bid);
            std::cout << "done" << std::endl;
        }

        //TODO: add config loag section

        std::cout << "Calibrate Magnetometer (y/n)? ";
        if(yn_answer()) {
            calibration::calibrateMagnetometer(seatrac, std::cout, std::cin, false);
        }
        std::cout << "Calibrate Accelerometer (y/n)? ";
        if(yn_answer()) {
            calibration::calibrateAccelerometer(seatrac, std::cout, std::cin, false);
        }

        std::cout << "Save Settings to permanent EEPROM memory (y/n)? ";
        if(yn_answer()) {
            std::cout << "Saving Settings... ";
            command::settings_save(seatrac);
            std::cout << "done" << std::endl;
        }

        std::cout << "Beacon setup complete" << std::endl;
        }

        std::cout << std::endl << "Setup another beacon (y/n)? ";
        cont = yn_answer();
    }
}


        // std::cout << "Beacon Settings:";
        // std::cout << "Change Beacon Settings (y/n): ";
        // std::cout << "Upload from config file (u) or enter manually (m)? ";
        // std::cout << "Path to Config File (blank for default './seatrac_logger_config.toml'): ";

// int main(int argc, char *argv[])
// {
//     std::string serial_port = "/dev/ttyUSB0";
//     MyDriver seatrac(serial_port);
//     command::status_config_set(seatrac, (STATUS_BITS_E)0x0);

//     int action;
//     std::cout << "Running Seatrac Modem Calibration:" << std::endl
//               << "Which calibration procedure would you like to execute?" << std::endl
//               << "\t1) Magnetometer Calibration" << std::endl
//               << "\t2) Accelerometer Calibration" << std::endl
//               << "\t3) Both Magnetometer and Acceleromter Calibration" << std::endl
//               << "\t4) Dry run - cal settings only saved to RAM, not EEPROM" << std::endl
//               << "\t5 or more) Exit" << std::endl
//               << "Enter a number: ";
//     scanf("%d", &action);
//     switch(action) {
//         case 1: {
//             calibration::calibrateMagnetometer(seatrac, std::cout, std::cin, true);
//         } break;
//         case 2: {
//             calibration::calibrateAccelerometer(seatrac, std::cout, std::cin, true);
//         } break;
//         case 3: {
//             calibration::calibrateMagnetometer(seatrac, std::cout, std::cin, true);
//             calibration::calibrateAccelerometer(seatrac, std::cout, std::cin, true);
//         } break;
//         case 4: {
//             calibration::calibrateMagnetometer(seatrac, std::cout, std::cin, false);
//             calibration::calibrateAccelerometer(seatrac, std::cout, std::cin, false);
//         } break;
//         default: {
//             std::cout << "Exiting Seatrac Modem Calibration" << std::endl;
//         } break;
//     }

//     return 0;
// }
