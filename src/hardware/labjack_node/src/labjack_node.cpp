#include <ros/ros.h>
#include <LJM_Utilities.h>
#include "labjack_node/LabJackState.h"
#include "labjack_node/ReadDIO.h"
#include "labjack_node/WriteDIO.h"

#include <memory>
#include <iostream>

const double EPSILON = 1e-3;

using namespace labjack_node;

class LabJack {
    public:
        LabJack(const std::string& serial_number) {
            handle = OpenOrDie(LJM_dtT7, LJM_ctANY, "LJM_idANY");
        }
        ~LabJack() {
            CloseOrDie(handle);
        }
        bool readDIO(const std::string& name) {
            double value;
            int err = LJM_eReadName(handle, name.c_str(), &value);
            ErrorCheck(err, "LJM_eReadName");
            return value > EPSILON;
        }
        void writeDIO(const std::string& name, bool value) {
            double val = value ? 1.0 : 0.0;
            int err = LJM_eWriteName(handle, name.c_str(), val);
            ErrorCheck(err, "LJM_eWriteName");
        }
        bool readDIO(unsigned char pin_num) {
            std::string pin_name;
            pin_name = "DIO" + std::to_string(pin_num);
            return readDIO(pin_name);
        }
        void writeDIO(unsigned char pin_num, bool value) {
            std::string pin_name;
            pin_name = "DIO" + std::to_string(pin_num);
            writeDIO(pin_name, value);
        }
    private:
        int handle;
};

std::unique_ptr<LabJack> labjack_device;

bool read_dio(ReadDIO::Request &req,
              ReadDIO::Response &res) {
    res.value = labjack_device->readDIO(req.pin_id);
    return true;
}

bool write_dio(WriteDIO::Request &req,
               WriteDIO::Response &res) {
    labjack_device->writeDIO(req.pin_id, req.value);
    return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "labjack");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");

    labjack_device = std::unique_ptr<LabJack>(new LabJack("0"));

    // Turn off all pins on startup.
    for (int i=0; i<23; ++i) {
        labjack_device->writeDIO(i, false);
    }

    ros::ServiceServer read_dio_service = n.advertiseService("/labjack/read_dio/", read_dio);
    ros::ServiceServer write_dio_service = n.advertiseService("/labjack/write_dio/", write_dio);

    ros::spin();

    return 0;
}
