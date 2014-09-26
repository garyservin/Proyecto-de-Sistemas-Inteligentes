// This program drives the turtle along a circular way
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <turtlesim/Pose.h>

double radius;

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "circle");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  nh_.param("radius", radius, 1.0);

  // Create a publisher object.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "turtle1/cmd_vel", 1000);

    geometry_msgs::Twist msg;

    // Loop at 2Hz until the node is shut down.
    ros::Rate rate(10);
    while(ros::ok()) {
        ros::spinOnce();

        // v = r * w | w = v / r
        msg.linear.x = 1.0;
        msg.angular.z = msg.linear.x / radius;

        // Publish the message.
        pub.publish(msg);

        // Wait until it's time for another iteration.
        rate.sleep();
    }
}
