#include "ros/ros.h"
#include "project1/Wheels_velocity.h"
#include <geometry_msgs/TwistStamped.h>

#define R 0.07
#define L 0.200
#define W 0.169
#define T 5.0

#define CST T*60/R


class WheelPublisher {
private:
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher wheel_pub;

public:
    WheelPublisher() {
        cmd_vel_sub = n.subscribe("/cmd_vel", 1000, &WheelPublisher::cmdVelCallback, this);
        wheel_pub = n.advertise<project1::Wheels_velocity>("wheels_rpm", 1000);
    }

    void cmdVelCallback (const geometry_msgs::TwistStamped::ConstPtr& data) {
        project1::Wheels_velocity wheels;

        wheels.header.frame_id = "wheels_rpm";
        wheels.header.stamp = data->header.stamp;

        wheels.rpm_fl = CST * ( + data->twist.linear.x
                                + data->twist.linear.y
                                - data->twist.angular.z * (L + W));
        wheels.rpm_fr = CST * ( + data->twist.linear.x
                                - data->twist.linear.y
                                + data->twist.angular.z * (L + W));
        wheels.rpm_rl = CST * ( + data->twist.linear.x
                                - data->twist.linear.y
                                - data->twist.angular.z * (L + W));
        wheels.rpm_rr = CST * ( + data->twist.linear.x
                                + data->twist.linear.y
                                + data->twist.angular.z * (L + W));

        wheel_pub.publish(wheels);
    }
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "wheel_subscriber");

    WheelPublisher WheelPublisher;

    ros::spin();

    return 0;

}