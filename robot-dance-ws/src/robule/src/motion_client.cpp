#include "motion_client.h"

double MotionClient::turn(int degrees)
{
    return 0;
}
bool MotionClient::sendGoal(double angular, double linear, ros::Duration seconds)
{
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    // send a goal to the action
    robule_msgs::RouteGoal path;
    path.angular = angular;
    path.linear = linear;
    path.seconds = seconds;

    ac.sendGoal(path);
    return true;
}

bool MotionClient::moveCircle(double diameter, ros::Duration duration)
{
    double circ = M_PI * diameter;
    double lin_vel = circ / duration.sec;
    double ang_vel = (2 * M_PI) / duration.sec;
    sendGoal(ang_vel, lin_vel, duration);

    return true;
}

bool MotionClient::moveLine(double length, ros::Duration duration)
{
    double lin_vel = length / duration.sec;
    sendGoal(0.0, lin_vel, duration);

    return true;
}

bool MotionClient::moveArc(double diameter, int totalArcs, int moveArcs, ros::Duration duration, int direction)
{
    double circ = M_PI * diameter;
    double distance = circ * (static_cast<double>(moveArcs) / static_cast<double>(totalArcs));
    
    double lin_vel = distance / duration.sec;
    double ang_vel = copysign(direction, ((2 * M_PI) / duration.sec));
    sendGoal(ang_vel, lin_vel, duration);

    return true;
}

bool MotionClient::moveOval(double width, double length, ros::Duration duration)
{
    double circ = M_PI * width;
    double straight = length - width;
    double totalDist = circ + (2 * straight);

    ros::Duration circDuration = ros::Duration((circ / totalDist) * duration.sec); 
    ros::Duration straightDuration = ros::Duration(((straight * 2) / totalDist) * duration.sec); 

    double lin_vel = totalDist / duration.sec;
    
    return true;
}

bool MotionClient::moveHeart(double width, ros::Duration duration)
{
    return true;
}

void MotionClient::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{

}
    
tf2::Quaternion MotionClient::getTargetQuat(int degrees)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robule2");   
    MotionClient client;

    std::string heart("heart");
    std::string oval("oval");
    std::string circle("circle");

    // if(heart.compare(argv[1]) == 0)
    // {
    //     client.moveHeart();
    // }
    if(oval.compare(argv[1]) == 0)
    {
        client.moveOval(2, 4, ros::Duration(20));
    }
    if(circle.compare(argv[1]) == 0)
    {
        client.moveCircle(2, ros::Duration(20));
    }

    return 0;
}