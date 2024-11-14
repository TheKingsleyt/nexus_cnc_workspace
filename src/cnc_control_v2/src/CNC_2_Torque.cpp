#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include <serial/serial.h>
#include <iostream>
#include <sstream>

class CNC2TorqueNode
{
public:
    CNC2TorqueNode()
    {
        try {
            // Set up serial communication on /dev/ttyACM1
            serial_port.setPort("/dev/ttyACM1");
            serial_port.setBaudrate(115200);
            serial_port.setTimeout(serial::Timeout::simpleTimeout(1000));
            serial_port.open();
            ROS_INFO("Connected to /dev/ttyACM1 for CNC2 Torque control.");
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

        // Subscribe to the torque sensor topic
        torque_sub = nh.subscribe("torque_sensor_data", 10, &CNC2TorqueNode::listenerCallback, this);
    }

    ~CNC2TorqueNode()
    {
        if (serial_port.isOpen()) {
            serial_port.close();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber torque_sub;
    serial::Serial serial_port;

    void listenerCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
    {
        // Assuming Tx, Ty, and Tz are the first three data elements in the message
        int tx = msg->data[0];
        int ty = msg->data[1];
        int tz = msg->data[2];
        ROS_INFO("Received torques - Tx: %d, Ty: %d, Tz: %d", tx, ty, tz);

        // Construct G-code commands based on Tx, Ty, and Tz
        std::ostringstream gcode_x, gcode_y, gcode_z;
        gcode_x << "G01 X" << tx << " F500\n";
        gcode_y << "G01 Y" << ty << " F500\n";
        gcode_z << "G01 Z" << tz << " F500\n";

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
    ros::init(argc, argv, "cnc2torque_node");
    CNC2TorqueNode cnc2_torque_node;
    ros::spin();
    return 0;
}

