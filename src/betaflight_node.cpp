#include <ros/ros.h>

#include <std_msgs/UInt16.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <agiros_msgs/Command.h>

#include <cmath>

#include "msp/Client.hpp"
#include "msp/msp_msg.hpp"

#include <agilib/bridge/thrust_map.hpp>

double deg2rad(const double deg) {
    return deg/180.0 * M_PI;
}

double rad2deg(const double rad) {
    return rad/M_PI * 180.0;
}

class BetaflightNode {
private:
    ros::NodeHandle nh;
    msp::client::Client *client;

    agi::ThrustMap thrustmap;

    double voltage;
    double quad_mass;
    double max_rate;

    ros::Publisher imu_pub;
    ros::Publisher battery_pub;

    ros::Subscriber command_sub;

public:
    BetaflightNode() {
        nh = ros::NodeHandle("~");
        client = new msp::client::Client();
    }

    ~BetaflightNode() {
        delete client;
    }

    msp::client::Client& bf() const {
        return *client;
    }

    void setup() {
        std::string device;
        std::string command_topic;
        std::string thrust_map_file;
        int baudrate = 1000000;
        if(nh.getParam("device", device)) {
            if(!nh.getParam("baudrate", baudrate)) {
                ROS_WARN("Parameter 'baudrate' not set. Using default baudrate of %i", baudrate);
            }
            ROS_INFO("Connecting to FCU at %s", device.c_str());
        }
        else {
            ROS_ERROR("Parameter 'device' not set.");
        }

        if (!nh.getParam("command_topic", command_topic)) {
            ROS_ERROR("Parameter 'command_topic' not set.");
        }

        //load thrust map
        if (!nh.getParam("thrust_map_file", thrust_map_file)) {
            ROS_ERROR("Parameter 'thrust_map_file' not set.");
        } else if (!thrustmap.load(thrust_map_file)) {
            ROS_ERROR("Failed to load thrust map!.");
        } else {
            ROS_INFO("Successfully loaded thrust map.");
        }

        if (!nh.getParam("max_rate", max_rate)) {
            ROS_ERROR("Parameter 'max_rate' not set.");
        }

        client->start(device, uint(baudrate));

        imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);
        battery_pub = nh.advertise<sensor_msgs::BatteryState>("/battery",10);

        command_sub = nh.subscribe(command_topic, 1, &BetaflightNode::commandCallBack, this); // AERT1234


    }

    void onImu(const msp::msg::RawImu &imu) {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "betaflight";

        imu_msg.linear_acceleration.x = imu.acc[0];
        imu_msg.linear_acceleration.y = imu.acc[1];
        imu_msg.linear_acceleration.z = imu.acc[2];

        imu_msg.angular_velocity.x = imu.gyro[0];
        imu_msg.angular_velocity.y = imu.gyro[1];
        imu_msg.angular_velocity.z = imu.gyro[2];

        imu_msg.orientation_covariance[0] = -1.0;

        imu_pub.publish(imu_msg);

    }

    void onBattery(const msp::msg::Analog &analog) {
        voltage = analog.vbat;

        sensor_msgs::BatteryState battery;
        battery.header.stamp = ros::Time::now();
        battery.voltage = analog.vbat;
        battery.current = analog.amperage;

        battery_pub.publish(battery);
    }

    void commandCallBack(const agiros_msgs::Command& cmd) {
        msp::msg::SetRawRc rc(msp::FirmwareVariant::BAFL);
        // ROS_INFO("%f", voltage);
        const double thrust_out = thrustmap.map(cmd.collective_thrust, voltage);

        // Catch mapping error for zero thrust.
        const double thrust_apply =
            thrust_out * 1000 + 1000;

        // Convert to unsigned int and clamp in valid SBUS range.
        const uint16_t throttle = std::clamp((int)thrust_apply, 1000, 2000);

        const uint16_t roll = std::clamp((int)(rad2deg(cmd.bodyrates.x) / max_rate * 500 + 1500), 1000, 2000);
        const uint16_t pitch = std::clamp((int)(rad2deg(cmd.bodyrates.y) / max_rate * 500 + 1500), 1000, 2000);
        const uint16_t yaw = std::clamp((int)(rad2deg(cmd.bodyrates.z) / max_rate * 500 + 1500), 1000, 2000);
        // std::cout << roll << ", " << pitch << ", " << yaw << ", " << throttle << ", " << thrust_out << std::endl;
        rc.channels.resize(6, 1500);
        rc.channels[0] = roll; //roll
        rc.channels[1] = pitch; //pitch
        rc.channels[2] = throttle; //yaw
        rc.channels[3] = yaw; //throttle
        //std::cout << rc.channels[0]  << ", "<< rc.channels[1] << ", " << rc.channels[2]  << ", "<< rc.channels[3] << std::endl;

        client->sendMessageNoWait(rc);

    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "Betaflight Bridge");

    BetaflightNode node;

    node.setup();
    ROS_INFO("Betaflight bridge is ready!");

    node.bf().subscribe(&BetaflightNode::onImu, &node, 0.001);
    node.bf().subscribe(&BetaflightNode::onBattery, &node, 0.1);

    ros::spin();

    ros::shutdown();

    return 0;
}