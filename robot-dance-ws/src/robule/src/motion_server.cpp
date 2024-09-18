#include "motion_server.h"

MotionServer::MotionServer()
    : nh_priv_("~"), as_(nh_, "path",  boost::bind(&MotionServer::motion_callback, this, _1), false)
{
    //Init gazebo ros turtlebot3 node
    ROS_INFO("TurtleBot3 Simulation Node Init");
    auto ret = init();
    as_.start();
    ROS_ASSERT(ret);
}

bool MotionServer::init()
{
    // initialize publishers
    cmd_vel_pub_   = nh_priv_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // initialize subscribers
    odom_sub_ = nh_.subscribe("odom", 10, &MotionServer::odomMsgCallBack, this);

    return true;
}

tf2::Quaternion MotionServer::getTargetQuat(int degrees)
{
    // convert input degrees to radians
    double targetYaw = degrees * (M_PI / 180);

    // declare new quaternion for just storing rotation
    tf2::Quaternion q_rot;
    // set rotation to targetYaw
    q_rot.setRPY(0, 0, targetYaw);

    // declare new quaternion for result rotation
    tf2::Quaternion q_new;
    // apply target rotation to current heading
    q_new = q_rot * q_orig;

    // return the normalised the quaternion
    return q_new.normalize();
}

void MotionServer::motion_callback(const robule_msgs::RouteGoalConstPtr &goal)
{
    bool success = true;
    geometry_msgs::Twist dirMsg;
    dirMsg.angular.z = goal->angular;
    dirMsg.linear.x = goal->linear;

    // set current time for reference
    ros::Time startTime = ros::Time::now();

    ROS_INFO("Got a goal!\n");
    // move for time

    while(ros::Time::now() - startTime < goal->seconds)
    {
        feedback_.countdown = (ros::Time::now() - startTime);
        as_.publishFeedback(feedback_);
        ROS_INFO_STREAM(feedback_.countdown << "\n");
        cmd_vel_pub_.publish(dirMsg);
        ros::spinOnce();
    }

    if(success)
    {
        result_.result = ros::Duration(0);
        dirMsg.angular.z = 0;
        dirMsg.linear.x = 0;
        cmd_vel_pub_.publish(dirMsg);
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}

double MotionServer::turn(int degrees)
{
    // get target quaternion rotation and convert it to a matrix
    tf2::Matrix3x3 matrix(getTargetQuat(degrees));

    // these need to be declared because matrix.RPY uses by reference params
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    // get the roll, pitch, yaw from the matrix
    matrix.getRPY(roll, pitch, yaw);

    // only care about returning yaw value
    return yaw;
}

void MotionServer::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf2::convert(msg->pose.pose.orientation, q_orig);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robule");

    MotionServer onlyDoesMotion;
    ros::spin();

    return 0;
}