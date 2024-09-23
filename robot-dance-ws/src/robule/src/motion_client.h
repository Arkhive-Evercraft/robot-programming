#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "robule_msgs/RouteAction.h"
#include "std_msgs/Duration.h"

class MotionClient
{
public:
    enum direction
    {
        left = -1,
        right = 1,
    };

    double turn(int degrees);
    bool sendGoal(double angular, double linear, ros::Duration seconds);

    bool moveCircle(double diameter, ros::Duration duration);
    bool moveLine(double length, ros::Duration duration);
    bool moveArc(double diameter, int totalArcs, int moveArcs, ros::Duration duration, int direction);

    bool moveOval(double width, double length, ros::Duration duration);
    bool moveHeart(double width, ros::Duration duration);

    MotionClient() = default;
    ~MotionClient() = default;

protected:
    void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    tf2::Quaternion getTargetQuat(int degrees);
    tf2::Quaternion q_orig;

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<robule_msgs::RouteAction> ac{"path", true};
};