#ifndef OBSTACLE_VELOCITY_COST_FUNCTION_H
#define OBSTACLE_VELOCITY_COST_FUNCTION_H

#include <base_local_planner/trajectory_cost_function.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <cmath>

#include <dynamic_obstacle_detector/DynamicObstacles.h>
#include <nav_msgs/Odometry.h>


using namespace std;
namespace base_local_planner {

class ObstacleVelocityCostFunction : public TrajectoryCostFunction{
private:
    /* data */
    vector <double> obs_vx,obs_vy;
    vector <double> obs_x,obs_y;
    double robot_vx_, robot_vy_,robot_x_,robot_y_, robot_v_;
    ros::Subscriber rdv, rdv_KF, amcl_sub_, rosaria_sub_;
        

public:
    ObstacleVelocityCostFunction(/* args */);
    ~ObstacleVelocityCostFunction();
    bool prepare();
    double scoreTrajectory(Trajectory &traj);
    void callback(const gazebo_msgs::ModelStates::ConstPtr& model_states);
    double GetAngle(double x1, double y1, double x2, double y2);
    double ComupteCI(double vh, double vr, double angelb, double anglef,double distance);
    double getScale();
    void setvelocitytopic(string veloctiy_topic);

    void KfPredCallback(const dynamic_obstacle_detector::DynamicObstacles::ConstPtr& pre_obstacle_states);

    void AmclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& robot_pose);

    void RosariaPoseCallback(const nav_msgs::Odometry::ConstPtr& robot_state);

};

}
#endif