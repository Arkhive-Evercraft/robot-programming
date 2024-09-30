#include "motion_client.h"

double MotionClient::turn(int degrees)
{
    return 0;
}
bool MotionClient::sendGoal(robule_msgs::RouteGoal path)
{
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    // send a goal to the action
    ac.sendGoal(path);
    ROS_INFO("Goal");
    bool goalTimeout = !ac.waitForResult(path.seconds + ros::Duration(1));
    if(goalTimeout)
    {
        ROS_WARN("Goal has timed out. Aborting.\n");
        return false;
    }
    return true;
}

ros::Duration MotionClient::beatDuration(double bpm, double beats)
{
    return ros::Duration((60.00 / bpm) * beats);
}

robule_msgs::RouteGoal MotionClient::createCircle(double diameter, ros::Duration duration)
{
    robule_msgs::RouteGoal circleGoal;
    double circ = M_PI * diameter;
    circleGoal.linear = circ / duration.sec;
    circleGoal.angular = (2 * M_PI) / duration.sec;
    circleGoal.seconds = duration;

    return circleGoal;
}

robule_msgs::RouteGoal MotionClient::createLine(double length, ros::Duration duration)
{
    robule_msgs::RouteGoal lineGoal;
    lineGoal.linear = length / duration.sec;
    lineGoal.angular = 0.0;
    lineGoal.seconds = duration;

    return lineGoal;
}

robule_msgs::RouteGoal MotionClient::createArc(double diameter, int totalArcs, int moveArcs, ros::Duration duration, direction dir)
{
    // fraction of the circle to travel
    double circFraction = (static_cast<double>(moveArcs) / (static_cast<double>(totalArcs)));
    // time it would take to complete the whole circle at speed
    double fullCircTime = duration.sec / circFraction;
    // length of the arc
    double distance = M_PI * diameter * circFraction;
    double lin_vel = distance / duration.sec;
    double ang_vel = std::copysign((2 * M_PI) / fullCircTime, dir);

    robule_msgs::RouteGoal arcGoal;
    arcGoal.linear = lin_vel;
    arcGoal.angular = ang_vel;
    arcGoal.seconds = duration;

    return arcGoal;
}

bool MotionClient::moveOval(double width, double length, ros::Duration duration)
{
    // calculate the half circumference
    double semiCirc = M_PI * width * 0.5;
    // calculate the length of the straight edge (total length - (2 * radius))
    double straight = length - width;
    // calculate the distance of the whole shape ((2 * radius) + (2 * straight edges))

    // calculate what percentage of the duration is given to curves ((2 * half circles) / total distance)
    ros::Duration semiCircDuration = ros::Duration((semiCirc / ((2 * semiCirc) + (2 * straight))) * static_cast<double>(duration.sec)); 
    
    //calculate what percentage of the duration is given to straight edges ((2 * straight edges) / total distance)
    ros::Duration straightEdgeDuration = ros::Duration((straight / ((2 * semiCirc) + (2 * straight))) * static_cast<double>(duration.sec)); 
    // calculate linear velocity required to complete the total distance
    double lin_vel = ((2 * semiCirc) + (2 * straight)) / duration.sec;

    std::array<robule_msgs::RouteGoal, 2> goals;
    // define semi-circle 
    goals[0] = createArc(width, 4, 2, semiCircDuration, direction::left);

    // define straight-edge goal
    goals[1] = createLine(straight, straightEdgeDuration);


    for(int i = 0; i < 4; ++i)
    {
        sendGoal(goals[i % 2]);
        bool goalTimeout = !ac.waitForResult(goals[i % 2].seconds + ros::Duration(1));
        if(goalTimeout)
        {
            ROS_WARN("Goal has timed out. Aborting.\n");
            return false;
        }
    }
    
    return true;
}

bool MotionClient::moveHeart(double width, ros::Duration duration)
{
    return true;
}

bool MotionClient::moveWiggle(double width, double length, ros::Duration duration)
{
    // calculate the half circumference
    double semiCirc = M_PI * width * 0.5;
    // calculate the number of turns in the line
    const int turns = 4;
    // calculate the total distance of the line
    double distance = turns * semiCirc;
    ros::Duration semiCircDuration = ros::Duration(duration.sec / turns); 
    
    std::array<robule_msgs::RouteGoal, turns> goals;
    // define semi-circle 
    goals[0] = createArc(width, 4, 2, semiCircDuration, direction::left);
    goals[1] = createArc(width, 4, 2, semiCircDuration, direction::right);

    for(int i = 0; i < turns; ++i)
    {
        sendGoal(goals[i % 2]);
        bool goalTimeout = !ac.waitForResult(goals[i % 2].seconds + ros::Duration(1));
        if(goalTimeout)
        {
            ROS_WARN("Goal has timed out. Aborting.\n");
            return false;
        }
    }

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
    ros::init(argc, argv, "rob_dance");   
    MotionClient client;

    double bpm = std::stof(argv[1]);
    std::array<robule_msgs::RouteGoal, 30> goals;
    bool goalSuccess = true;

    // turn 45deg left
    goals[0] = client.createArc(0, 8, 1, client.beatDuration(bpm, 8), MotionClient::direction::left);
    // move straight
    goals[1] = client.createLine(1, client.beatDuration(bpm, 4));

    // goals [2] - [7]
    for(int i = 2; i < 8; i+=2) 
    {
        // turn 90deg right
        goals[i] = client.createArc(0, 4, 1, client.beatDuration(bpm, 4), MotionClient::direction::right);
        // move straight
        goals[i + 1] = client.createLine(1, client.beatDuration(bpm, 4));
    }
    // turn 135deg left
    goals[8] = client.createArc(0, 8, 3, client.beatDuration(bpm, 8), MotionClient::direction::left);
    // move semicircle
    goals[9] = client.createArc(1, 4, 2, client.beatDuration(bpm, 8), MotionClient::direction::left);
    // turn 90deg left
    goals[10] = client.createArc(0, 4, 1, client.beatDuration(bpm, 4), MotionClient::direction::left);
    // move straight
    goals[11] = client.createLine(1, client.beatDuration(bpm, 4));
    // turn 90deg right
    goals[12] = client.createArc(0, 4, 1, client.beatDuration(bpm, 4), MotionClient::direction::right);
    // move semicircle
    goals[13] = client.createArc(1, 4, 2, client.beatDuration(bpm, 8), MotionClient::direction::right);
    // turn 90deg right
    goals[14] = client.createArc(0, 4, 1, client.beatDuration(bpm, 4), MotionClient::direction::right);

    for(int i = 0; i < goals.size() && goalSuccess; ++i)
    {
        goalSuccess = client.sendGoal(goals[i]);
    }
    

    return 0;
}
