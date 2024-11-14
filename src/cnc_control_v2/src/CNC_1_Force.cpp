#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include <serial/serial.h>
#include <iostream>
#include <sstream>

class CNC1ForceNode
{
public:
    CNC1ForceNode()
    {
        try {
            // Set up serial communication on /dev/ttyACM0
            serial_port.setPort("/dev/ttyACM0");
            serial_port.setBaudrate(115200);
            serial_port.setTimeout(serial::Timeout::simpleTimeout(1000));
            serial_port.open();
            ROS_INFO("Connected to /dev/ttyACM0 for CNC1 Force control.");
        } catch (const serial::IOException& e) {
            ROS_ERROR("Failed to open serial port: %s", e.what());
            ros::shutdown();
            return;
        }

        // Ensure the port is open
        if (!serial_port.isOpen()) {
            ROS_ERROR("Serial port not open.");
            ros::shutdown();
            return;
        }

        // Subscribe to the force sensor topic
        force_sub = nh.subscribe("force_sensor_data", 10, &CNC1ForceNode::listenerCallback, this);
    }

    ~CNC1ForceNode()
    {
        if (serial_port.isOpen()) {
            serial_port.close();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber force_sub;
    serial::Serial serial_port;

    void listenerCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
    {
        // Assuming Fx, Fy, and Fz are the first three data elements in the message
        int fx = msg->data[0];
        int fy = msg->data[1];
        int fz = msg->data[2];
        ROS_INFO("Received forces - Fx: %d, Fy: %d, Fz: %d", fx, fy, fz);

        // Construct G-code commands based on Fx, Fy, and Fz
        std::ostringstream gcode_x, gcode_y, gcode_z;
        gcode_x << "G01 X" << fx << " F500\n";
        gcode_y << "G01 Y" << fy << " F500\n";
        gcode_z << "G01 Z" << fz << " F500\n";

        // Write G-code commands to the serial port
        serial_port.write(gcode_x.str());
        ROS_INFO("Sent X-axis G-code command: %s", gcode_x.str().c_str());
        
        serial_port.write(gcode_y.str());
        ROS_INFO("Sent Y-axis G-code command: %s", gcode_y.str().c_str());
        
        serial_port.write(gcode_z.str());
        ROS_INFO("Sent Z-axis G-code command: %s", gcode_z.str().c_str());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cnc1force_node");
    CNC1ForceNode cnc1_force_node;
    ros::spin();
    return 0;
}

