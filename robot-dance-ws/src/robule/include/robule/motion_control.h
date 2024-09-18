#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/server/simple_action_server.h>

class DirControl
{
public:
    enum Direction
    {
        slightLeft = 45,
        left = 90,
        sharpLeft = 120,

        slightRight = -45,
        right = -90,
        sharpRight = -120,

        around = 180
    };

    double turn(int degrees);
    DirControl();
    ~DirControl() = default;

private:
    // ROS NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;

    // ROS Topic Publishers
    ros::Publisher cmd_vel_pub_;

    // ROS Topic Subscribers
    ros::Subscriber laser_scan_sub_;
    ros::Subscriber odom_sub_;

    tf2::Quaternion q_orig;

    // actionlib::SimpleActionServer<geometry_msgs::Twist> motionServer;

    bool init();
    void updatecommandVelocity(double linear, double angular);
    void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    tf2::Quaternion getTargetQuat(int degrees);
};