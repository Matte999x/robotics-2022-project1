#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "project1/ResetOdometry.h"
#include <dynamic_reconfigure/server.h>
#include <project1/integrationMethodConfig.h>
#include "parameters.h"


// robot parameters
using namespace parameters;

class OdometryNode {
private:
    ros::NodeHandle nodeHandle;
    ros::Subscriber sub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::ServiceServer resetOdometryService;
    dynamic_reconfigure::Server<project1::integrationMethodConfig> dynamicReconfigureServer;
    dynamic_reconfigure::Server<project1::integrationMethodConfig>::CallbackType dynamicReconfigureCallback;

    bool setup;
    enum IntegrationMethod { Euler, RungeKutta } integrationMethod;

    ros::Time previous_time;
    ros::Time current_time;
    long int previous_ticks[4];
    float wheel_speeds[4];
    double x, y, theta;
    geometry_msgs::TwistStamped velocity;


public:
    OdometryNode () {
        setup = true;

        // subscribe to /wheel_states topic
        sub = nodeHandle.subscribe("/wheel_states", 1000, &OdometryNode::wheelDataCallback, this);

        // advertise /cmd_vel and /odom topics
        cmd_vel_pub = nodeHandle.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);
        odom_pub = nodeHandle.advertise<nav_msgs::Odometry>("/odom", 1000);

        // advertise reset_odometry service
        resetOdometryService = nodeHandle.advertiseService("reset_odometry", &OdometryNode::resetOdometry, this);

        // get the initial values of the odometry integration from the parameter server
        nodeHandle.getParam("/x0", x);
        nodeHandle.getParam("/y0", y);
        nodeHandle.getParam("/theta0", theta);

        // setup the dynamic reconfigure for the odometry integration method
        integrationMethod = IntegrationMethod::Euler;
        dynamicReconfigureCallback = boost::bind(&OdometryNode::dynamicReconfigureIntegrationMethod, this, _1, _2);
        dynamicReconfigureServer.setCallback(dynamicReconfigureCallback);
    }

    void wheelDataCallback (const sensor_msgs::JointState::ConstPtr& data) {
        current_time = data->header.stamp;
        if (setup) {
            // executed only at first message received
            setup = false;
            previous_time = current_time;
            for (int i=0; i<4; i++)
                previous_ticks[i] = data->position[i];
        } else {
            // compute time step
            double deltaTime = (current_time - previous_time).toSec();
            previous_time = current_time;

            // compute wheel speeds
            double delta_ticks[4];
            for(int i=0; i<4; i++) {
                delta_ticks[i] = data->position[i]-previous_ticks[i];
                previous_ticks[i] = data->position[i];
                wheel_speeds[i] =  delta_ticks[i] / deltaTime * 2 * M_PI / N / T;
            }

            // compute robot velocity (local reference frame)
            velocity.header.stamp = current_time;
            velocity.header.frame_id = "robot velocity";
            velocity.twist.linear.x  = R/4 * (+ wheel_speeds[0]
                                              + wheel_speeds[1]
                                              + wheel_speeds[2]
                                              + wheel_speeds[3]);
            velocity.twist.linear.y  = R/4 * (- wheel_speeds[0]
                                              + wheel_speeds[1]
                                              + wheel_speeds[2]
                                              - wheel_speeds[3]);
            velocity.twist.linear.z  = 0;
            velocity.twist.angular.x = 0;
            velocity.twist.angular.y = 0;
            velocity.twist.angular.z = R / 4 / L_W * (- wheel_speeds[0]
                                                      + wheel_speeds[1]
                                                      - wheel_speeds[2]
                                                      + wheel_speeds[3]);

            // publish robot velocity on /cmd_vel topic
            cmd_vel_pub.publish(velocity);

            // choose integration method for odometry
            float angle;            
            switch (integrationMethod) {
            case IntegrationMethod::Euler:
                angle = theta;
                break;
            case IntegrationMethod::RungeKutta:
                angle = theta + velocity.twist.angular.z * deltaTime / 2.0;
                break;
            default:
                angle = theta;
            }

            // compute odometry
            x = x + (velocity.twist.linear.x * cos(angle) - velocity.twist.linear.y * sin(angle)) * deltaTime;
            y = y + (velocity.twist.linear.x * sin(angle) + velocity.twist.linear.y * cos(angle)) * deltaTime;
            theta = theta + velocity.twist.angular.z * deltaTime;

            // create rotation quaternion for publishing TF and odometry
            tf2::Quaternion rotation_quaternion;
            rotation_quaternion.setRPY(0, 0, theta);

            // broadcast TF odom->base_link
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
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
            odometryMsg.header.frame_id = "odom";
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
            odom_pub.publish(odometryMsg);
        }
    }

    bool resetOdometry(project1::ResetOdometry::Request &request, project1::ResetOdometry::Response &response) {
        // callback for reset_odometry service
        x = request.x;
        y = request.y;
        theta = request.theta;
        ROS_INFO("Resetting odometry to (%f, %f, %f)", x, y, theta);
        return true;
    }

    void dynamicReconfigureIntegrationMethod(project1::integrationMethodConfig &config, uint32_t level) {
        // callback for integradion method dynamic reconfigure
        if (config.odometry_integration_method >= 0 && config.odometry_integration_method < 2)
            integrationMethod = (IntegrationMethod)config.odometry_integration_method;

        switch (integrationMethod) {
        case IntegrationMethod::Euler:
            ROS_INFO("Integration method set to Euler");
            break;
        case IntegrationMethod::RungeKutta:
            ROS_INFO("Integration method set to Runge Kutta");
            break;
        default:
            ROS_INFO("Unimplemented integration method");
            break;
        }        
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "bag_sub");

    OdometryNode odometryNode;

    ros::spin();

    return 0;
}
