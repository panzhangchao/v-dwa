#include <dwa_local_planner/compute_c.h>

namespace dwa_local_planner{


compute_c::compute_c()
{
    ros::NodeHandle nh_;
    // obs_sub_ = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",100,boost::bind(&compute_c::obstacleCallback, this, _1));
    dy_obs_sub_ = nh_.subscribe<dynamic_obstacle_detector::DynamicObstacles>("/dynamic_obstacles", 100, boost::bind(&compute_c::dynamic_obstacleCallback, this, _1));
    set_callback = false;

}

double compute_c::computeC(geometry_msgs::PoseStamped& robot_pose_, const geometry_msgs::Twist& robot_vel, double angle)
{
    set_bool(true);
    ros::spinOnce();
    int size_obs = obstacle_velx_.size();
    double robot_vel_x, robot_vel_y;
    robot_vel_x = robot_vel.linear.x * cos(angle);
    robot_vel_y = robot_vel.linear.x * sin(angle);
    double CI_sum = 0;
    // cout << "the obstacle size is: " << size_obs << endl;
    if(size_obs > 0)
    {
        for(int i = 0;i<size_obs;i++)
        {
            double vel_obs = sqrt(pow(obstacle_vely_[i], 2) + pow(obstacle_velx_[i], 2));
            double vel_rob = robot_vel.linear.x;
            double angelb = GetAngle(obstacle_velx_[i], obstacle_vely_[i], robot_pose_.pose.position.x - obstacle_pose_x_[i], robot_pose_.pose.position.y - obstacle_pose_y_[i]);
            double angelf = GetAngle(robot_vel_x, robot_vel_y, obstacle_pose_x_[i]-robot_pose_.pose.position.x, obstacle_pose_y_[i] - robot_pose_.pose.position.y);
            double distance =sqrt(pow(robot_pose_.pose.position.x - obstacle_pose_x_[i], 2) + pow(robot_pose_.pose.position.y - obstacle_pose_y_[i],2));

            double CI = (vel_rob * cos(angelf) + vel_obs * cos(angelb))/distance;
            CI_sum = CI + CI_sum;
        }
        
        return CI_sum;
    }

    else
    {
        return -100000;
    }
    

}
// 计算两个向量夹角，从x1,y1 到x2,y2。逆时针为正，顺时针为负
double compute_c::GetAngle(double x1, double y1, double x2, double y2)
{
    double TheNorm;
    double rho;
    double angle;
    TheNorm = sqrt(x1*x1+y1*y1)*sqrt(x2*x2+y2*y2);
    // cout << "the norm is:" << TheNorm <<endl;
    rho = x1*y2-x2*y1;
    angle = acos((x1*x2+y1*y2)/TheNorm);
    if(TheNorm==0)
    {
        return -10;
    }
    if(rho<0){
        return -angle;
    }
    else{
        return angle;
    }
}

int compute_c::obstacleCallback(const gazebo_msgs::ModelStates::ConstPtr &modelstates)
{   
    
    if(!set_callback)
    {
        return 0;
    }
    obstacle_pose_x_.clear();
    obstacle_pose_y_.clear();
    obstacle_velx_.clear();
    obstacle_vely_.clear();
    int num;
    num = modelstates->name.size();
    for (int i=0; i<num; i++)
    {
        if((fabs(modelstates->twist[i].linear.x)>0.05||fabs(modelstates->twist[i].linear.y)>0.05) && modelstates->name[i] != "/")
        {
            obstacle_pose_x_.push_back(modelstates->pose[i].position.x);
            obstacle_pose_y_.push_back(modelstates->pose[i].position.y);
            obstacle_velx_.push_back(modelstates->twist[i].linear.x);
            obstacle_vely_.push_back(modelstates->twist[i].linear.y);
        }
    }
    set_bool(false);
    return 0;

}


void compute_c::set_bool(bool flag)
{
    set_callback = flag;
}

compute_c::~compute_c()
{
    
}

int compute_c::dynamic_obstacleCallback(const dynamic_obstacle_detector::DynamicObstacles::ConstPtr& pre_obstacle_states)
{
    if(!set_callback)
    {
        return 0;
    }
    obstacle_pose_x_.clear();
    obstacle_pose_y_.clear();
    obstacle_velx_.clear();
    obstacle_vely_.clear();
    int num;
    num = pre_obstacle_states->obstacles.size();
    for (int i=0; i<num; i++)
    {
        if((fabs(pre_obstacle_states->obstacles[i].velocity.x)>0.05||fabs(pre_obstacle_states->obstacles[i].velocity.y)>0.05))
        {
            obstacle_pose_x_.push_back(pre_obstacle_states->obstacles[i].position.x);
            obstacle_pose_y_.push_back(pre_obstacle_states->obstacles[i].position.y);
            obstacle_velx_.push_back(pre_obstacle_states->obstacles[i].velocity.x);
            obstacle_vely_.push_back(pre_obstacle_states->obstacles[i].velocity.y);
        }
    }
    set_bool(false);
    return 0;
}



};