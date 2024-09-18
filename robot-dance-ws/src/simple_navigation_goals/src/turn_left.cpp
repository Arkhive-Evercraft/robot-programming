#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

tf2::Quaternion q_orig;
ros::Publisher direction_pub;
ros::Subscriber odom_sub;

void getOdom(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf2::convert(msg->pose.pose.orientation , q_orig);
    ROS_INFO_STREAM("w: " << q_orig.getW() << "x: " << q_orig.getX() 
    << "y: " << q_orig.getY() << "z: " << q_orig.getZ() << "\n");
}

void turnLeft()
{
    tf2::Quaternion q_rot, q_new;
    q_rot.setRPY(0, 0, -1.5708);
    q_new = q_rot * q_orig;  // Calculate the new orientation
    q_new.normalize();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    direction_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    odom_sub = n.subscribe("odom", 1000, getOdom);
    
    double speed = 0.3;
    int degrees = 90;
    double radians = 0.0;
    double roll = 0.0;
    double pitch = 0.0; 
    double yaw = 0.0;

    geometry_msgs::Twist dirMsg;
    dirMsg.linear.x = speed;
    direction_pub.publish(dirMsg);
    
    int count = 0;
    while (ros::ok())
    {
        tf2::Matrix3x3 m(q_orig);
        m.getRPY(roll, pitch, yaw);

        if(count == 20)
        {
            radians = degrees * (M_PI / 180);
        }

        dirMsg.angular.z = 0.5 * (radians - yaw);
        direction_pub.publish(dirMsg);
        ros::spinOnce();
        count++;
        loop_rate.sleep();
    }

    return 0;
}