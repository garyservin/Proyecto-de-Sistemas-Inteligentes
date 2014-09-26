// This program drives the turtle along a circular way
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <turtlesim/Pose.h>

bool first_time = true;
double radius;
bool goal_reached = false;
bool heading_reached = false;

turtlesim::PoseConstPtr g_pose;
turtlesim::Pose g_goal;

bool hasReachedGoal()
{
  return fabsf(g_pose->x - g_goal.x) < 0.1 && fabsf(g_pose->y - g_goal.y) < 0.1;
}

bool hasReachedHeading()
{
  return fabsf(g_pose->theta - g_goal.theta) < 0.01;
}

// A callback function.  Executed each time a new pose
// message arrives.
void poseCallback(const turtlesim::PoseConstPtr& pose){
    g_pose = pose;
    if(first_time){
        ROS_INFO_NAMED("circle", "First pose received");
        first_time = false;
        g_goal.x = pose->x + radius;
        g_goal.y = pose->y;
        g_goal.theta = 1.57;
    }

    if(hasReachedGoal()){
        ROS_INFO_ONCE_NAMED("circle", "Goal reached");
        goal_reached = true;
    }

    if(hasReachedHeading()){
        ROS_INFO_ONCE_NAMED("circle", "Heading reached");
        heading_reached = true;
    }
}

int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "circle");
  ros::NodeHandle nh;

  ros::NodeHandle nh_("~");

  nh_.param("radius", radius, 1.0);

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000,
    &poseCallback);

  // Create a publisher object.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "turtle1/cmd_vel", 1000);

    geometry_msgs::Twist msg;

    // Loop at 2Hz until the node is shut down.
    ros::Rate rate(10);
    while(ros::ok()) {
        ros::spinOnce();

        if(!first_time){
            if(goal_reached && heading_reached){
                // v = r * w | w = v / r
                ROS_INFO_ONCE_NAMED("circle", "Circling");
                msg.linear.x = 1.0;
                msg.angular.z = msg.linear.x / radius;
            }else if(goal_reached){
                ROS_INFO_ONCE_NAMED("circle", "Rotating");
                msg.linear.x = 0.0;
                msg.angular.z = 1.0;
            }else{
                ROS_INFO_ONCE_NAMED("circle", "Moving");
                msg.linear.x = 1.0;
                msg.angular.z = 0.0;
            }

            ROS_DEBUG_NAMED("circle", "Setting speed to %.2f, %.2f", msg.linear.x, msg.angular.z);

            // Publish the message.
            pub.publish(msg);

        }
        // Wait until it's time for another iteration.
        rate.sleep();
    }
}
