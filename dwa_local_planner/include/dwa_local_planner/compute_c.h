#ifndef _COMPUTE_C_H_
#define _COMPUTE_C_H_

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <math.h>
#include <cmath>

#include <dynamic_obstacle_detector/DynamicObstacles.h>

using namespace std;
namespace dwa_local_planner{

class compute_c
{
private:
    /* data */
    // ros::NodeHandle nh_;
    geometry_msgs::PoseStamped robot_pose_;
    vector <double> obstacle_pose_x_, obstacle_pose_y_;
    vector <double> obstacle_velx_, obstacle_vely_;
    bool set_callback;
    ros::Subscriber obs_sub_;
    ros::Subscriber robot_sub_;
    ros::Subscriber dy_obs_sub_;
    bool flag_callback;

public:
    compute_c(/* args */);
    ~compute_c();
    int obstacleCallback(const gazebo_msgs::ModelStates::ConstPtr &modelstates);
    double computeC(geometry_msgs::PoseStamped& robot_pose, const geometry_msgs::Twist& robot_vel, double angle);
    double GetAngle(double x1, double y1, double x2, double y2);
    void set_bool(bool flag);
    
    int dynamic_obstacleCallback(const dynamic_obstacle_detector::DynamicObstacles::ConstPtr& pre_obstacle_states);
};
};

#endif