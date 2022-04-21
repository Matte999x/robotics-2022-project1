#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
// #include <project1/parametersConfig.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define r 0.07
#define L 0.200
#define W 0.169
#define T 5.0
#define N 42.0

#define cnst1 r/4
#define cnst2 r/(4*(L+W))


//using namespace sensor_msgs;

class BagSubscriber {

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Subscriber sub2;
    ros::Publisher cmd_vel_pub;
    ros::Publisher calculated_pose_pub;

    ros::Time previous_time;
    ros::Time current_time;

    geometry_msgs::TwistStamped velocity;
    geometry_msgs::PoseStamped poseStamped;

    tf2_ros::TransformBroadcaster tf_broadcaster;

    float current_wheel_velocity[4];
    long int previous_ticks[4];
    float current_movement_velocity[4];
    double x, y, theta;

    bool setup, setup_starting_position, mode = false;


public:
    BagSubscriber () {
        setup = true;
        setup_starting_position = false;

        sub = n.subscribe("/wheel_states", 1000, &BagSubscriber::wheelDataCallback, this);
        sub2 = n.subscribe("/robot/pose", 1000, &BagSubscriber::robotCallback, this);

        cmd_vel_pub = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);
        calculated_pose_pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);

        n.getParam("/x", x);
        n.getParam("/y", y);
        n.getParam("/theta", theta);
    }

    void robotCallback (const geometry_msgs::PoseStamped::ConstPtr& data) {
        if (setup_starting_position) {

        ROS_INFO("Passato da setup");
            setup_starting_position = false;

            x = data->pose.position.x;
            y = data->pose.position.y;

            tf2::Quaternion q(data->pose.orientation.x,
                              data->pose.orientation.y,
                              data->pose.orientation.z,
                              data->pose.orientation.w);

            tf2::Matrix3x3 m(q);

            double deadbeef;
            m.getRPY(deadbeef, deadbeef, theta);
        }

//        ROS_INFO("x : %f %f", data->pose.position.x, poseStamped.pose.position.x);
//        ROS_INFO("y : %f %f", data->pose.position.y, poseStamped.pose.position.y);
//        ROS_INFO("z : %f %f", data->pose.position.z, poseStamped.pose.position.z);
//
//        ROS_INFO("x : %f %f", data->pose.orientation.x, poseStamped.pose.orientation.x);
//        ROS_INFO("y : %f %f", data->pose.orientation.y, poseStamped.pose.orientation.y);
//        ROS_INFO("z : %f %f", data->pose.orientation.z, poseStamped.pose.orientation.z);
//        ROS_INFO("w : %f %f", data->pose.orientation.w, poseStamped.pose.orientation.w);
    }

    void wheelDataCallback (const sensor_msgs::JointState::ConstPtr& data) {
        current_time = data->header.stamp;
        if (setup) {
            setup = false;
            previous_time = current_time;
            for (int i=0; i<4; i++) {
                previous_ticks[i] = data->position[i];
//                ROS_INFO("%s", data->name[i].c_str());
            }
        } else {
            // compute time step
            double deltaTime = (current_time - previous_time).toSec();
            previous_time = current_time;

            double constant = 2 * M_PI / N / T / deltaTime;

            // compute wheel speeds
            double delta_ticks[4];
            for(int i=0; i<4; i++) {
                delta_ticks[i] = data->position[i]-previous_ticks[i];
                previous_ticks[i] = data->position[i];
                current_wheel_velocity[i] = constant * delta_ticks[i];
            }

            // compute robot velocity (local reference frame)
            compute_velocity();

            // compute odometry
            float angle;
            if (mode == false)
                angle = theta;  // Euler method
            else
                angle = theta + velocity.twist.angular.z * deltaTime / 2.0;  // Runge-Kutta method (2nd order)

            x = x + (velocity.twist.linear.x * cos(angle) + velocity.twist.linear.y * sin(angle)) * deltaTime;
            y = y + (velocity.twist.linear.x * sin(angle) - velocity.twist.linear.y * cos(angle)) * deltaTime;
            theta = theta + velocity.twist.angular.z * deltaTime;

            // create rotation quaternion for publishing TF and odometry
            tf2::Quaternion rotation_quaternion;
            rotation_quaternion.setRPY(0, 0, theta);

            // broadcast TF odom->base_link
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "world";
            odom_trans.child_frame_id = "base_link";
            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation.x = rotation_quaternion.x();
            odom_trans.transform.rotation.y = rotation_quaternion.y();
            odom_trans.transform.rotation.z = rotation_quaternion.z();
            odom_trans.transform.rotation.w = rotation_quaternion.w();
            tf_broadcaster.sendTransform(odom_trans);

            // publish odometry
            nav_msgs::Odometry odometryMsg;
            odometryMsg.header.stamp = current_time;
            odometryMsg.header.frame_id = "world";
            odometryMsg.pose.pose.position.x = x;
            odometryMsg.pose.pose.position.y = y;
            odometryMsg.pose.pose.position.z = 0.0;
            odometryMsg.pose.pose.orientation.x = rotation_quaternion.x();
            odometryMsg.pose.pose.orientation.y = rotation_quaternion.y();
            odometryMsg.pose.pose.orientation.z = rotation_quaternion.z();
            odometryMsg.pose.pose.orientation.w = rotation_quaternion.w();
            odometryMsg.child_frame_id = "base_link";
            odometryMsg.twist.twist.linear.x = velocity.twist.linear.x;
            odometryMsg.twist.twist.linear.y = velocity.twist.linear.y;
            odometryMsg.twist.twist.linear.z = 0.0;
            odometryMsg.twist.twist.angular.x = 0.0;
            odometryMsg.twist.twist.angular.y = 0.0;
            odometryMsg.twist.twist.angular.z = velocity.twist.angular.z;
            calculated_pose_pub.publish(odometryMsg);
        }

    }

    void compute_velocity() {

        /* Matrice di trasformazione presa da:
           "Global Localization and Position Tracking of Automatic Guided Vehicles using passive RFID Technology"
           University of Dortmund */

        velocity.header.stamp = current_time;
        velocity.header.frame_id = "robot velocity";

        velocity.twist.linear.x  = cnst1 * (+ current_wheel_velocity[1]
                                            + current_wheel_velocity[0]
                                            + current_wheel_velocity[2]
                                            + current_wheel_velocity[3]);
        velocity.twist.linear.y  = cnst1 * (- current_wheel_velocity[1]
                                            + current_wheel_velocity[0]
                                            - current_wheel_velocity[2]
                                            + current_wheel_velocity[3]);
        velocity.twist.linear.z  = 0;

        velocity.twist.angular.x = 0;
        velocity.twist.angular.y = 0;
        velocity.twist.angular.z = cnst2 * (+ current_wheel_velocity[1]
                                            - current_wheel_velocity[0]
                                            - current_wheel_velocity[2]
                                            + current_wheel_velocity[3]);

        cmd_vel_pub.publish(velocity);

    }
/*
    void compute_position () {

        odometry.header.frame_id = "world";
        odometry.header.stamp = current_time;
        odometry.child_frame_id = "robot";

        odometry.pose.pose.position.x += + velocity.twist.linear.x * delta_minutes * cos(rotation_angle) + velocity.twist.linear.y * delta_minutes * sin(rotation_angle);
        odometry.pose.pose.position.y += - velocity.twist.linear.y * delta_minutes * cos(rotation_angle) + velocity.twist.linear.x * delta_minutes * sin(rotation_angle);
        odometry.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, rotation_angle);

        odometry.pose.pose.orientation.x = q.x();
        odometry.pose.pose.orientation.y = q.y();
        odometry.pose.pose.orientation.z = q.z();
        odometry.pose.pose.orientation.w = q.w();

        odometry.twist.twist.angular.x = 0;
        odometry.twist.twist.angular.y = 0;
        odometry.twist.twist.angular.z = rotation_angle;

        calculated_pose_pub.publish(odometry);
        // transform_broadcaster.sendTransform(odometry);

    }
*/
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "bag_sub");

    // dynamic_reconfigure::Server<project1::parametersConfig> dynServer;
    // dynamic_reconfigure::Server<project1::parametersConfig>::CallbackType f;
//
    // f = boost::bind()

    BagSubscriber bagSub;
    ros::spin();
    return 0;
}
