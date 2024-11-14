#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
// Additional libraries and definitions for Net F/T communication...

class ForceSensorPublisher {
public:
    ForceSensorPublisher() {
        pub_ = nh_.advertise<std_msgs::Int32MultiArray>("force_sensor_data", 10);
    }

    void publishData() {
        std_msgs::Int32MultiArray msg;
        // Populate msg with force/torque data, e.g., msg.data = {Fx, Fy, Fz, Tx, Ty, Tz}
        pub_.publish(msg);
        ros::spinOnce();
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "netft_force_sensor_publisher");
    ForceSensorPublisher sensor_publisher;
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        sensor_publisher.publishData();
        loop_rate.sleep();
    }
    return 0;
}
