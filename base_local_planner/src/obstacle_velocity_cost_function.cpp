#include <base_local_planner/obstacle_velocity_cost_function.h>
#include <ros/ros.h>

// FILE* file2 = fopen("/home/lovebc/Experiment/3/data.txt", "w+");
namespace base_local_planner{
    ObstacleVelocityCostFunction::ObstacleVelocityCostFunction(){
        setvelocitytopic("");
    }

    ObstacleVelocityCostFunction::~ObstacleVelocityCostFunction(){

    }
    bool ObstacleVelocityCostFunction::prepare(){
        return true;   
    }


    void ObstacleVelocityCostFunction::AmclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& robot_pose)
    {
        // std::cout << "in the amcl callback" << std::endl;
        robot_x_ = robot_pose->pose.pose.position.x;
        robot_y_ = robot_pose->pose.pose.position.y;

        
    }

    void ObstacleVelocityCostFunction::callback(const gazebo_msgs::ModelStates::ConstPtr& model_states){
        double vx,vy;
        int num=model_states->name.size();
        
        // obs_vx.clear();
        // obs_vy.clear();
        // obs_x.clear();
        // obs_y.clear();
        for (int i=0;i<num;i++)
        { 
        //    if((fabs(model_states->twist[i].linear.x)>0.05||fabs(model_states->twist[i].linear.y)>0.05) && model_states->name[i] != "/")
        //    {
        //         obs_vx.push_back(model_states->twist[i].linear.x);
        //         obs_vy.push_back(model_states->twist[i].linear.y);
        //         obs_x.push_back(model_states->pose[i].position.x);
        //         obs_y.push_back(model_states->pose[i].position.y);
        //    }
            if(model_states->name[i] == "/")
            {
                // cout << "the model name is:" <<model_states->name[i] << endl;
                robot_vx_ = model_states->twist[i].linear.x;
                robot_vy_ = model_states->twist[i].linear.y;
                robot_x_ = model_states->pose[i].position.x;
                robot_y_ = model_states->pose[i].position.y;
            }
        }
    }

    void ObstacleVelocityCostFunction::KfPredCallback(const dynamic_obstacle_detector::DynamicObstacles::ConstPtr& pre_obstacle_states)
    {
        int obstacle_size = pre_obstacle_states->obstacles.size();

        obs_x.clear();
        obs_y.clear();
        obs_vx.clear();
        obs_vy.clear();
        // std::cout << "in the I want callback" << std::endl;
        for(int i = 0; i < obstacle_size; i++)
        {
            // std::cout <<" the vx is: "<< pre_obstacle_states->obstacles[i].velocity.x << std::endl;
            obs_vx.push_back(pre_obstacle_states->obstacles[i].velocity.x);
            obs_vy.push_back(pre_obstacle_states->obstacles[i].velocity.y);

            obs_x.push_back(pre_obstacle_states->obstacles[i].position.x);
            obs_y.push_back(pre_obstacle_states->obstacles[i].position.y);
        }
        //  std::cout << " in the KF callback" << std::endl;
    }

    void ObstacleVelocityCostFunction::RosariaPoseCallback(const nav_msgs::Odometry::ConstPtr& robot_state)
    {
        robot_x_ = robot_state->pose.pose.position.x;
        robot_y_ = robot_state->pose.pose.position.y;

        double angle = robot_state->pose.pose.orientation.z;
        robot_v_ = robot_state->twist.twist.linear.x;
        robot_vx_ = robot_v_ * cos(angle);
        robot_vy_ = robot_v_ * sin(angle);
        // std::cout << " in the RosAria callback" << std::endl;

    }
    void ObstacleVelocityCostFunction::setvelocitytopic(string velocity_topic)
    {
        ros::NodeHandle nh_;
        string robot_topic="/gazebo/model_states";
        velocity_topic = "/dynamic_obstacles";
        string robot_pose_topic = "/amcl_pose";
        string RobotStateTopic = "RosAria/pose";
        
        
        // -rdv是读取gazebo中机器人的速度, rdv_KF是读取预测的障碍物状态信息, amcl_sub_是读取蒙特卡洛定位的机器人位置
        // 1 rosaria 订阅是机器人码盘的机器人速度和位置，是相对于odom的坐标系的。


        // rdv = nh_.subscribe<gazebo_msgs::ModelStates>(robot_topic, 1, boost::bind(&ObstacleVelocityCostFunction::callback,this,_1));
        
        rdv_KF = nh_.subscribe<dynamic_obstacle_detector::DynamicObstacles>(velocity_topic, 1, boost::bind(&ObstacleVelocityCostFunction::KfPredCallback, this, _1));
        
        // amcl_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robot_pose_topic, 1, boost::bind(&ObstacleVelocityCostFunction::AmclCallback, this, _1));
        
        rosaria_sub_ = nh_.subscribe<nav_msgs::Odometry>(RobotStateTopic, 1, boost::bind(&ObstacleVelocityCostFunction::RosariaPoseCallback, this, _1));

    }

    /* DWA中是采样轨迹，先锋是差速，所以没有y方向速度，*/
    double ObstacleVelocityCostFunction::scoreTrajectory(Trajectory &traj){
        double cost_one = 0;
        double cost_total = 0;
        double px, py, pth;
        // std::cout << "in the v score Trajectory" << std::endl;
        double obs_distance;
        double threshold;
        
        for(unsigned int i=0;i<traj.getPointsSize();i++)
        {
             
            cost_one=0;
        
            if(i==traj.getPointsSize()-1)
            {
                traj.getPoint(i,px,py,pth); 
                double min_obs_distance = 100000;
                for(int j=0;j<obs_vx.size();j++)
                {
                    obs_distance = sqrt(pow(obs_x[j]-px,2)+pow(obs_y[j]-py,2));
                    
                    // double obs_x_ = obs_x[j] + obs_vx[j]*1.7;
                    // double obs_y_ = obs_y[j] + obs_vy[j]*1.7; 

                    double obs_x_ = obs_x[j];
                    double obs_y_ = obs_y[j]; 


                    // 考虑如何把障碍物速度，角度等和检测距离有关
                    double obs_v = sqrt(pow(obs_vx[j], 2) + pow(obs_vy[j], 2));

                    // std::cout << "the obstacle vx is:" << obs_vx[j] << "vy is: "<< obs_vy[j] << "x is: " << obs_x_<< "y is: "<< obs_y_<< std::endl;
                    // std::cout << "the robot vx is: " << robot_vx_ << "vy is: " << robot_vy_ << "x is: " << robot_x_ << "y is: " << robot_y_ << std::endl;
                    double alpha = GetAngle(robot_vx_, robot_vy_, obs_x_ - robot_x_, obs_y_ - robot_y_);
                    double phi = GetAngle(obs_vx[j], obs_vy[j], robot_x_ - obs_x_, robot_y_ - obs_y_);
                    // double dis_colid = robot_v_ * cos(alpha) + obs_v * cos(phi);
                    double dis_colid = 0.55 * cos(alpha) + obs_v * cos(phi);
                    // fprintf(file2, "robot pose: %f, %f, robot vel: %f, %f, obs pose: %f, %f, obs vel: %f, %f, \
                    //         angle: al: %f, ph: %f dis_colid: %f\n", robot_x_, robot_y_, robot_vx_, robot_vy_, obs_x_, obs_y_, \
                    //         obs_vx[j], obs_vy[j], alpha, phi, dis_colid);
                    if(dis_colid < 0)
                    {
                        continue;
                    }
                    else
                    {
                        threshold = 3.16 * dis_colid;
                    }

                    // threshold = 3.16 * (0.55 + obs_v);
                    // 后续可以改为在连线上的投影
                    if(obs_distance < min(max(threshold, 2.0),4.0))  // 原来是3.5
                    // if(obs_distance < 3.5)
                    {
                        double th = GetAngle(obs_vx[j],obs_vy[j],px-obs_x_,py-obs_y_);
                        cost_one = cost_one+cos(th)+1;                        
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
    
        cost_one = exp(4*cost_one);
        return cost_one;

    }

    /* 计算两个向量夹角，从x1,y1到x2,y2逆时针为正，顺时针为负*/
    double ObstacleVelocityCostFunction::GetAngle(double x1, double y1, double x2, double y2)
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
        if(angle<0)
        {
            angle = -angle;
        }

        if(rho<0){
            return -angle;
        }
        else{
            return angle;
        }
    }
    /*计算CI*/
    /*
    double ObstacleVelocityCostFunction::ComupteCI(double vh, double vr, double angelb, double angelf, double distance)
    {
        
        double H = (vh*cos(angelb)+vr*cos(angelf))/distance;
        double H_max = (vh+vr)/distance;
        double CI = exp(H/H_max);
        if(H<0){
            return 0;
        }
        else{
            return CI;
        }
    }
*/
    double ObstacleVelocityCostFunction::getScale()
    {
        return 1;
    }
}