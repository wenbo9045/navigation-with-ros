#include "rnav/local_planner/tri_planner_ros.h"

using namespace std;
namespace tri_local_planner{
  
  TRIPlannerROS::TRIPlannerROS() : initialized_(false),
      odom_helper_("odom"), receiveTraj(true),setup_(false),ifupdated(false),updateStartPose(false) { } 
        
  void TRIPlannerROS::initialize(
        string name,
        tf::TransformListener* tf, 
        localMap::localMap* costmap)
  {
    best_traj_pub = nn.advertise<geometry_msgs::PointStamped>("/best_traj",1);
    
    probe_pub = nn.advertise<visualization_msgs::MarkerArray>("/check_probe",1);
    
    marker_point_pub = nn.advertise<visualization_msgs::Marker>("visualize_points",10);
    angle_deviation_pub = nn.advertise<geometry_msgs::PointStamped>("/selected_point",10);
    if(! isInitialized())
    {
        ros::NodeHandle private_nh("~/"+name);
        g_plan_pub_=private_nh.advertise<nav_msgs::Path>("global_plan",1);
        l_plan_pub_=private_nh.advertise<nav_msgs::Path>("local_plan",1);
        tf_=tf;
        costmap_=costmap;
        costmap_->getRobotPose(current_pose_);

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      
      
      private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.15);
      private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.2);
      private_nh.param("orientation_tolerance", orientation_tolerance, 0.4);
      
      position_reached_check=false;
      initialized_ = true;
      recovery_counter=0;
      recovery_counter_add=0;
      vel_set_=0;
      count_=0;
      w_set=0;
      state="TRI";
      start_orientation=0.0;
      
    //速度采样参数设置
    private_nh.param("max_rot_vel", max_vel_th, 2.3);
    private_nh.param("min_rot_vel", min_vel_th, -2.3);
    private_nh.param("max_vel_x", max_vel_x, 0.7);
    private_nh.param("min_vel_x", max_vel_x, 0.0);
    private_nh.param("max_vel_y", max_vel_x, 0.0);
    private_nh.param("min_vel_y", max_vel_x, 0.0);
    private_nh.param("max_trans_vel", max_trans_vel, 0.7);
    private_nh.param("min_trans_vel", min_trans_vel, 0.0);
    private_nh.param("sim_time", sim_time_, 1.5);    
    private_nh.param("vx_samples", vx_samples, 3.0);
    private_nh.param("vtheta_samples", vth_samples, 20.0);

        
    //轨迹生成参数
    private_nh.param("sim_granularity", sim_granularity, 0.025);
    private_nh.param("angular_sim_granularity", angular_sim_granularity, 0.1);
    private_nh.param("global_frame_id", frame_id, std::string("map"));
    private_nh.param("publish_traj_pc", publish_traj_pc_, true);
    //轨迹发布 
    traj_cloud_ = new pcl::PointCloud<MapGridCostPoint>;
    traj_cloud_->header.frame_id = frame_id;
    traj_cloud_pub_.advertise(private_nh, "trajectory_cloud", 1);  
    //轨迹评分参数
    private_nh.param("path_distance_bias", path_distance_bias, 32.0);  
    private_nh.param("goal_distance_bias", goal_distance_bias, 64.0); 
    private_nh.param("occdist_scale", occdist_scale, 0.1);  
    pdist_scale_=(costmap_->getResolution()) * path_distance_bias * 0.5;
    gdist_scale_= (costmap_->getResolution()) * goal_distance_bias * 0.5;
    occdist_scale_=(costmap_->getResolution()) * occdist_scale;
    //调速参数
    private_nh.param("max_turn_radius", max_turn_radius, 0.8);
    private_nh.param("min_straight_points", min_straight_points, 16);
    private_nh.param("straight_orientation_tolerance", straight_orientation_tolerance, 10.0);  
    private_nh.param("diff_begin_tolerance", diff_begin_tolerance, 1.0);
    private_nh.param("diff_end_tolerance", diff_end_tolerance, 2.0);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
    
    goal_prepare_map(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    path_prepare_map(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    ROS_WARN("Base local planner initialized");
    
    point_set.header.frame_id = "/map";
    point_set.type = visualization_msgs::Marker::POINTS;
    point_set.scale.x = 0.1;
    point_set.scale.y = 0.1;
    point_set.color.b = 1.0;
    point_set.color.a = 1.0;
  }
  
bool TRIPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
 //ROS_INFO("local planner set size %d",plan.size());
 if(receiveTraj)
 {
  if(plan.size() == 0)  
  {
   updateStartPose=false;
  }
  else gl_plan_vector = plan;
 }
}
  
TRIPlannerROS::~TRIPlannerROS(){

}
  
bool TRIPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if ( ! costmap_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    if(gl_plan_vector.empty())
      return false;

    if(stop_times_!=0)
    {
     stop_times_--;
     ROS_ERROR("stop time %d",stop_times_);
     cmd_vel.linear.x = 0;
     cmd_vel.linear.y = 0;
     cmd_vel.linear.z = 0; 
     cmd_vel.angular.z = 0; 
     return true;
    }
   //poseStamped vector类型的；
   getTransformPlan(current_pose_,gl_plan_vector,transformed_plan); 
   //nav_msgs::Path类型的     
   publishPlan(transformed_plan,g_plan_pub_); 

    if (isPositionReached(current_pose_)) {
      state="Reached";
      recovery_counter=0;
      recovery_counter_add=0;
      vel_set_=0;
      w_set = 0;
      count_=0;
      //publish an empty plan
      std::vector<geometry_msgs::PoseStamped> local_plan;
      publishPlan(local_plan,g_plan_pub_);
      publishPlan(local_plan,l_plan_pub_);  
      if(stop_times_ == 0)
      computeVelocityCommandsStopRotate(cmd_vel);
    }
    else  if(!isRobotSafe(current_pose_))
    {  
        state="Stop";
        updateStartPose=false;
        ROS_INFO("collision detected");
        vel_set_=0;
        w_set=0;
        count_=0;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0; 
        cmd_vel.angular.z = 0; 
    }
    else if(isOrientationAjusted(current_pose_)){
      //ROS_ERROR("Ajust");
      state="Ajust";
      vel_set_=0;
      count_=0;
      recovery_counter=0;
      recovery_counter_add=0;
      //double cmd_vel_temp = 0.5 + fabs(start_orientation)/3.141592654*0.4;
      double cmd_vel_temp = 0.5;
      if(pre_state == "TRI" || pre_state == "Stop") 
	{stop_times_ = 5;ROS_WARN("STOP FIRST");}
      if(stop_times_==0)
      {
        if(start_orientation > 0){//-0.5
         if(start_orientation > 0.5)
         {
            adjust_w(w_set,-cmd_vel_temp,cmd_vel_temp,false);
         }
         else
         {
            w_set=-0.3;
         }
         cmd_vel.angular.z = w_set;
      }
      else{ //0.5
         //adjust_w(w_set,-cmd_vel_temp,cmd_vel_temp,true); 
         if(start_orientation < -0.5)
         {
            adjust_w(w_set,-cmd_vel_temp,cmd_vel_temp,true); 
         }
         else
         {
            w_set= 0.3;
         }
         cmd_vel.angular.z = w_set;  
      }
      ROS_INFO("Ajust Ajust Ajust Ajust w_set:%f",w_set);  
      }
    }

    /*else if(isPlanStraight(current_pose_,transformed_plan))
    {
        if(state=="Ajust")
        {
            vel_set_=0;
            count_=0;
        }
        state="STRAIGHT";
        recovery_counter=0;
        recovery_counter_add=0; 
        w_set=0;  
        vel_select(current_pose_,vel_set_);
        cmd_vel.linear.x=vel_set_;
        ROS_INFO("STRAIGHT vel:%.2f",vel_set_);
    }*/
    else{
        if(state=="Ajust")
        {
            vel_set_=0;
            count_=0;
        }
      state="TRI";
      recovery_counter=0;
      recovery_counter_add=0;
      if(pre_state =="Ajust" || pre_state == "Stop") 
      {stop_times_ = 5;ROS_WARN("STOP FROM AJUST");}
      if(stop_times_ == 0)
      {
        bool isOK=triComputeVelocityCommands(current_pose_,cmd_vel);
        if(!isOK){
        ROS_WARN_NAMED("tri_local_planner", "TRI planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishPlan(empty_plan,g_plan_pub_);
        publishPlan(empty_plan,l_plan_pub_);
      }  
      }       
    }
    pre_state = state;
    return true;
  }

bool TRIPlannerROS::getLocalPlanCheckingPoint()
{
 geometry_msgs::PoseStamped position_pre = transformed_plan[0];
 vector<geometry_msgs::PoseStamped> position_set;
 float dist;
 point_set.points.clear();
 position_set.push_back(transformed_plan[0]);
 for(int counter = 0; counter < transformed_plan.size(); ++counter)
 {
  dist = distance(position_pre,transformed_plan[counter]);
  if(dist>=0.3) 
  {
   position_set.push_back(transformed_plan[counter]);
   position_pre = transformed_plan[counter];
  }
 }
 
 geometry_msgs::Point pp;//用于可视化
 for(int i = 0; i < position_set.size(); ++i)
 {
  pp.x = position_set[i].pose.position.x;
  pp.y = position_set[i].pose.position.y;
  point_set.points.push_back(pp);//用于可视化
 }
 marker_point_pub.publish(point_set);
 
 
 geometry_msgs::PoseStamped selectedPose;
 bool updatedpoint = checkGoalSet(position_set,selectedPose);
 if(!updatedpoint)
 {
  geometry_msgs::PointStamped selected_point;
  selected_point.header.frame_id ="map";
  selected_point.point.x = selectedPose.pose.position.x;
  selected_point.point.y = selectedPose.pose.position.y;
  angle_deviation_pub.publish(selected_point);
  return true;
 }
 return false;
}

bool TRIPlannerROS::triComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose, geometry_msgs::Twist& cmd_vel) {
    
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
         
     tf::Stamped<tf::Pose> robot_vel;
     odom_helper_.getRobotVel(robot_vel); 
     
     //vel_set_=0.3;
     vel_select(global_pose,vel_set_); //在此处选择速度
     
     //速度采样
     vector<pair<double,double> > vel_samples_;   
     vel_sample(robot_vel,vel_samples_,vel_set_);  
   
     //轨迹生成
     vector<Trajectory> all_explored;
     paths_generate(vel_samples_,all_explored);
       

     //轨迹评分
     goal_prepare_map.resetPathDist();
     path_prepare_map.resetPathDist();
     ROS_INFO("TRAJ SIZE %d",transformed_plan.size());
     goal_prepare_map.setLocalGoal(*costmap_,transformed_plan,current_pose_,vel_set_*sim_time_*0.6);
     path_prepare_map.setTargetCells(*costmap_,transformed_plan);
     
     Trajectory result_traj_;result_traj_.cost_=255.0;
     score_paths(all_explored,result_traj_);
     
     cmd_vel.linear.x=result_traj_.xv_;
     cmd_vel.angular.z=result_traj_.thetav_; 
     
     traj_temp=result_traj_;
     w_set = traj_temp.thetav_;
     
     return true;
  }

void TRIPlannerROS::vel_select(tf::Stamped<tf::Pose> &global_pose,double& vel_set)
{
    double center_x=0.0,center_y=0.0,turn_radius=0.0;
    
    //if(isBegin(global_pose)&&!isEnd(global_pose))
    if(isBegin(global_pose))
    {
        //vel_set=0.2;
        adjust_vel(vel_set,0.1,0.4,true);//提速时应设为最小速度-0.1，因为计数器初始为0，会直接满足速度+0.1的条件
    }
    else if(isEnd(global_pose))
    {
        ROS_WARN("Ending %f",vel_set);
        adjust_vel(vel_set,0.3,0.6,false);
    }
    /*else if(state=="TRI" && circleLeastFit(transformed_plan,center_x,center_y,turn_radius))
    {
        adjust_vel(vel_set,0.4,0.4,true);
        adjust_vel(vel_set,0.4,0.4,false);
        //vel_set=0.3;
        ROS_INFO("turn vel:%.2f,turn_radius:%.2f",vel_set,turn_radius);
        
    }*/
    else if(state=="TRI" && getLocalPlanCheckingPoint())
    {
        //adjust_vel(vel_set,0.1,0.4,true);
        adjust_vel(vel_set,0.4,0.6,false);
        //vel_set=0.3;
        ROS_WARN("We should slow down");
        
    }
    else 
    {
        adjust_vel(vel_set,0.1,0.6,true);//不可以两个0.5，逻辑问题，两个0.5则速度只能是0.5
    }
}


void TRIPlannerROS::adjust_vel(double& vel_set,double min_vel,double max_vel,bool increase)
{
    if(vel_set >= max_vel)
    {
        vel_set = max_vel;
    }
    if(vel_set <= min_vel)
    {
        vel_set = min_vel;
    }
    if(increase)
    {
        if(vel_set < max_vel)
        {
                vel_set+=0.01;
        }
    }
    else
    {
        if(vel_set > min_vel)
        {
         vel_set-=0.01;
        }
    }
}
/*
从Adjust状态切换到isRobotSafe/直线需要将w_set置0
切换到TRI状态，则将选择的最优轨迹的角速度赋值给w_set
*/
void TRIPlannerROS::adjust_w(double& w_set,double min_w,double max_w,bool increase)
{
    if(increase)
    {
        if(w_set < max_w)
        {
            w_set+=0.1;
        }
    }
    else
    {
        if(w_set > min_w)
        {
            w_set-=0.1;
        }
    }
    w_set=std::min(0.5,w_set);
    w_set=std::max(-0.5,w_set);
}
 
/*
轨迹评分
*/ 
void TRIPlannerROS::score_paths(vector<Trajectory> all_explored,Trajectory& result_traj_)
{
    ROS_ASSERT(all_explored.size());
   // ROS_ERROR("Score paths");
    for(int i=0;i<all_explored.size();i++)
    {
        double px, py, pth;
        unsigned int cell_x, cell_y;
        double grid_dist,path_dist;
        double g_cost,o_cost,p_cost;

        Trajectory current_traj=all_explored[i];
        //ROS_ERROR("traj size: %d",current_traj.getPointsSize());
        for(unsigned int j=0;j<current_traj.getPointsSize();j++)
        {
            current_traj.getPoint(j, px, py, pth);
        //ROS_ERROR("A j: %f, px: %f py: %f, pth: %f",j,px,py,pth);
            if(!costmap_->worldToMap(px,py,cell_x,cell_y))
            {
                all_explored[i].cost_=255.0;
                continue;
            }
            //1.obstacle
            o_cost=costmap_->getCost(cell_x,cell_y);
            if(o_cost == localMap::LETHAL_OBSTACLE ||
               o_cost == localMap::INSCRIBED_INFLATED_OBSTACLE ||
               o_cost == localMap::NO_INFORMATION)
            {
                all_explored[i].cost_=255.0;
                continue;
            }
            //2.goal
            ROS_ASSERT((goal_prepare_map.map_).size());
            grid_dist=goal_prepare_map(cell_x,cell_y).target_dist;
            //3.path
            ROS_ASSERT((path_prepare_map.map_).size());
            path_dist=path_prepare_map(cell_x,cell_y).target_dist; 
            if (grid_dist == goal_prepare_map.obstacleCosts()||
                path_dist == path_prepare_map.obstacleCosts()||
                grid_dist == goal_prepare_map.unreachableCellCosts()||
                path_dist == path_prepare_map.unreachableCellCosts())
            {
                all_explored[i].cost_=-3.0;
                
                continue;
            }
   
            g_cost = grid_dist;
            p_cost = path_dist;   

        }
        all_explored[i].cost_=gdist_scale_*g_cost+occdist_scale_*o_cost+pdist_scale_*p_cost;
        if(all_explored[i].cost_>=0&&all_explored[i].cost_<=result_traj_.cost_){
            result_traj_=all_explored[i];
           // ROS_WARN("Get Best traj");
         }         
    }
    //ROS_ERROR("Score paths Over");
    //发布最优轨迹末端点
    int i=result_traj_.getPointsSize();
    //ROS_ERROR("SIZE: %d",i);
    geometry_msgs::PointStamped best_traj_end;
    if(i>0)
    {
    double p_x, p_y, p_th;
    //ROS_ERROR("B i: %f, px: %f py: %f, pth: %f",i-1,p_x,p_y,p_th);
    result_traj_.getPoint(i-1,p_x,p_y,p_th);
    best_traj_end.header.frame_id ="map";
    best_traj_end.point.x = p_x;
    best_traj_end.point.y = p_y;
    best_traj_pub.publish(best_traj_end);       
    } 
    //ROS_ERROR("Score paths PUBLISH Over");
}



void TRIPlannerROS::paths_generate(vector<pair<double,double> > vel_samples_,vector<Trajectory>& all_explored)
{
    ROS_ASSERT(vel_samples_.size()); 
    for(int index=0;index<vel_samples_.size();index++)
    {
        //ROS_INFO("Sample %f, %f", vel_samples_[index].first,vel_samples_[index].second);
        double vsample_x=vel_samples_[index].first;
        double vsample_th=vel_samples_[index].second;
        geometry_msgs::Vector3 pose;
        pose.x=current_pose_.getOrigin().getX();
        pose.y=current_pose_.getOrigin().getY();
	//方位角
        pose.z=tf::getYaw(current_pose_.getRotation());
        Trajectory traj;
        traj.cost_=-1;
        //计算时间步长
        double sim_time_distance = vsample_x * sim_time_;
        double sim_time_angle = fabs(vsample_th) * sim_time_;
        int num_steps=ceil(std::max(sim_time_distance / sim_granularity,
            sim_time_angle    / angular_sim_granularity));
        double dt = sim_time_ / num_steps;
        traj.time_delta_ = dt;
        //生成一个速度对对应的轨迹
        for (int i = 0; i < num_steps; ++i) 
        {   
            traj.addPoint(pose.x, pose.y, pose.z);
      
            geometry_msgs::Vector3 pose_temp;
            pose_temp.x=pose.x + vsample_x * cos(pose.z) * dt;
            pose_temp.y=pose.y + vsample_x * sin(pose.z) * dt;
            pose_temp.z=pose.z + vsample_th * dt;
            pose=pose_temp;
        }
        traj.xv_=vsample_x;
        traj.thetav_=vsample_th;
        all_explored.push_back(traj);
    }
    if(publish_traj_pc_)
    {   
        MapGridCostPoint pt;
        traj_cloud_->points.clear();
        traj_cloud_->width = 0;
        traj_cloud_->height = 0;
        std_msgs::Header header;
        pcl_conversions::fromPCL(traj_cloud_->header, header);
        header.stamp = ros::Time::now();
        traj_cloud_->header = pcl_conversions::toPCL(header);

        for(std::vector<Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {

            // Fill out the plan
            for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
            
                double p_x, p_y, p_th;
                t->getPoint(i, p_x, p_y, p_th);
                pt.x=p_x;
                pt.y=p_y;
                pt.z=0;
                pt.path_cost=p_th;
                pt.total_cost=t->cost_;
                traj_cloud_->push_back(pt);
            }
        }
        traj_cloud_pub_.publish(*traj_cloud_);
    }
    //ROS_ERROR("Path generated");
}

void TRIPlannerROS::vel_sample(tf::Stamped<tf::Pose> robot_vel,vector<pair<double,double> >& vel_samples_,double vel_set)
{
    costmap_->getRobotPose(current_pose_);
    if(vth_samples>0)
    {   
        pair<double,double > sample= make_pair(0.0,0.0);
        double dist=hypot(goal_pose.getOrigin().getX()-current_pose_.getOrigin().getX(),goal_pose.getOrigin().getY()-current_pose_.getOrigin().getY());
        double robot_vel_x=robot_vel.getOrigin().getX();
        double robot_vel_th=tf::getYaw(robot_vel.getRotation());
        //定速的vel_set_x 
        double vel_set_x = std::max(std::min(max_vel_x, dist / sim_time_), vel_set);

        //w的最大值和最小值
        if((isBegin(current_pose_)||isEnd(current_pose_))&&(!isOrientationAjusted(current_pose_)))
        {
            max_vel_th = vel_set_*(vel_set_*5);
            min_vel_th = -vel_set_*(vel_set_*5);
        }
        else
        {
            max_vel_th = robot_vel_th+0.4/0.5;
            min_vel_th = robot_vel_th-0.4/0.5;
        }
        
        double step=(max_vel_th-min_vel_th)/vth_samples;
        
        for(double i=min_vel_th;i<=max_vel_th;i+=step)
        {
         sample.first=vel_set_x;
         sample.second=i;
         vel_samples_.push_back(sample);
       }          
    }  
}
bool TRIPlannerROS::isBegin(tf::Stamped<tf::Pose> global_pose)
{
    
    double robot_x=global_pose.getOrigin().getX();
    double robot_y=global_pose.getOrigin().getY();
    if(!updateStartPose)
    {
        ROS_ERROR("uuuuuuuuuuuuuuuuuuuuuuuuuuu");
        start_pose_x=robot_x;
        start_pose_y=robot_y;
        updateStartPose=true; 
        ifupdated = true;
    }
    
    if(ifupdated)
    {
     double diff_begin=hypot(start_pose_x-robot_x, start_pose_y-robot_y);
     if(diff_begin<diff_begin_tolerance){
         ROS_ERROR("DIFF_BEGIN");
         //ifupdated = false;
         return true;
     }
     else
     {
      ifupdated = false;
      return false;
     }
    }
   return false;
}
bool TRIPlannerROS::isEnd(tf::Stamped<tf::Pose> global_pose)
{
    double robot_x=global_pose.getOrigin().getX();
    double robot_y=global_pose.getOrigin().getY();
  
    double goal_x = goal_pose.getOrigin().getX();
    double goal_y = goal_pose.getOrigin().getY();
    
    double diff_end=hypot(goal_x-robot_x, goal_y-robot_y);
    if(diff_end<diff_end_tolerance){
        ROS_INFO("DIFF_END");
        return true;
    }
    else
        return false;
}


/*
    最小二乘法拟合圆曲线，判断是否符合转弯条件
*/
bool TRIPlannerROS::circleLeastFit(const std::vector<geometry_msgs::PoseStamped>& new_plan, double &center_x, double &center_y, double &radius)
{
     center_x = 0.0f;
     center_y = 0.0f;
     radius = 0.0f;
     if (new_plan.size() < 3)
     {
         return false;
     }

     double sum_x = 0.0f, sum_y = 0.0f;
     double sum_x2 = 0.0f, sum_y2 = 0.0f;
     double sum_x3 = 0.0f, sum_y3 = 0.0f;
     double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

     int N = new_plan.size()/3;
     int M = new_plan.size();
     for (int i = 0; i < N; i++)
     {
         double x = new_plan[i].pose.position.x;
         double y = new_plan[i].pose.position.y;
         double x2 = x * x;
         double y2 = y * y;
         sum_x += x;
         sum_y += y;
         sum_x2 += x2;
         sum_y2 += y2;
         sum_x3 += x2 * x;
         sum_y3 += y2 * y;
         sum_xy += x * y;
         sum_x1y2 += x * y2;
         sum_x2y1 += x2 * y;
     }

     double C, D, E, G, H;
     double a, b, c;

     C = N * sum_x2 - sum_x * sum_x;
     D = N * sum_xy - sum_x * sum_y;
     E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
     G = N * sum_y2 - sum_y * sum_y;
     H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
     a = (H * D - E * G) / (C * G - D * D);
     b = (H * C - E * D) / (D * D - G * C);
     c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

     center_x = a / (-2);
     center_y = b / (-2);
     radius = sqrt(a * a + b * b - 4 * c) / 2;
     //ROS_INFO("radius:%f",radius);
     
     double a_x=new_plan[0].pose.position.x, a_y=new_plan[0].pose.position.y;
     double b_x=new_plan[M].pose.position.x, b_y=new_plan[M].pose.position.y;
     double c_x=new_plan[M/2].pose.position.x, c_y=new_plan[M/2].pose.position.y;
     double ab=hypot(a_x-b_x,a_y-b_y);
     double ac=hypot(a_x-c_x,a_y-c_y);
     double bc=hypot(b_x-c_x,b_y-c_y);
     //ROS_INFO("fabs(ac+bc-ab):%f",fabs(ac+bc-ab));
     if((radius < max_turn_radius)&&fabs(ac+bc-ab)>0.05) //添加附加条件以排除一段直线上的一个小圆弧
        return true;
     else
        return false;
}


bool TRIPlannerROS::isPlanStraight(tf::Stamped<tf::Pose> global_pose, const std::vector<geometry_msgs::PoseStamped>& new_plan)
{   // return true;
    double orientation_tolerance = straight_orientation_tolerance;
    double local_goal_x = new_plan.back().pose.position.x;
    double local_goal_y = new_plan.back().pose.position.y;
    double x = global_pose.getOrigin().x(),
           y = global_pose.getOrigin().y(),
           yaw = tf::getYaw(global_pose.getRotation()); 
    double end_orientation=get_orientation(global_pose,local_goal_x,local_goal_y);
    
/*    
   for(std::vector<geometry_msgs::PoseStamped>::const_iterator iter = new_plan.begin(); iter!=new_plan.end(); iter++)
  {
   if((x-(*iter).pose.position.x)*(x-(*iter).pose.position.x)+(y-(*iter).pose.position.y)*(y-(*iter).pose.position.y)>1.0*1.0)
   {
    local_goal_x = (*iter).pose.position.x;
    local_goal_y = (*iter).pose.position.y;
    break;
   }
  }*/
    
    if(fabs(end_orientation)>orientation_tolerance)
    {
        //ROS_INFO("xxxxxxxxxx  %f",end_orientation);
        return false;
    }
      
    double planLength=sqrt((local_goal_x - x)*(local_goal_x - x)+(local_goal_y - y)*(local_goal_y - y));

  
///////////////////////判断是否是直线//////////////////////
      buff.clear();                          
      int steps=planLength/20;
      int straight_num_tolerence = min_straight_points;
      for (unsigned int i = 0,k=0; i < new_plan.size(),k<20; ++i,++k) 
      {
          double temp_x=new_plan[i].pose.position.x;
          double temp_y=new_plan[i].pose.position.y;
          if((temp_x - x)*(temp_x - x)+(temp_y - y)*(temp_y - y)>(k*steps)*(k*steps))
          {
                Position_temp.x=temp_x;
                Position_temp.y=temp_y;
                buff.push_back(Position_temp);
                //ROS_INFO("temp_x: %f,temp_y: %f",temp_x,temp_y);
          }
      } 
      int max_point_num=maxPoints(buff);
      //ROS_INFO("max_point_num: %i",max_point_num);
      
     int M = new_plan.size();
     double a_x=new_plan[0].pose.position.x, a_y=new_plan[0].pose.position.y;
     double b_x=new_plan[M].pose.position.x, b_y=new_plan[M].pose.position.y;
     double c_x=new_plan[M/2].pose.position.x, c_y=new_plan[M/2].pose.position.y;
     double ab=hypot(a_x-b_x,a_y-b_y);
     double ac=hypot(a_x-c_x,a_y-c_y);
     double bc=hypot(b_x-c_x,b_y-c_y);
     //ROS_INFO("fabs(ac+bc-ab):%f",fabs(ac+bc-ab));

      if(max_point_num > straight_num_tolerence)
      {
            //ROS_INFO("max_point_num: %i",max_point_num);
            return true;
      }
      else if(fabs(ac+bc-ab)<0.03)//若一段路径的总长度与起点和终点距中点的距离之和相差不大，则判定为直线
      {
            //ROS_INFO("Straight--fabs(ac+bc-ab):%.3f",fabs(ac+bc-ab));
            return true;
      }
       else 
         return false;                
}

int TRIPlannerROS::maxPoints(vector<Position_float> &points) {
        int len=points.size();
        if(len==1||len==2) return len;
        int ret=0;
        for(int i=0;i<len;i++)
        {
            int curmax=1;
            map<double,int> mp;
            int v=1;
            int h=1;
            int same=0;
            int mpk=0;
            for(int j=0;j<len;j++)
            {
                if(j!=i)
                {
                    double x1=points[i].x-points[j].x;
                    double y1=points[i].y-points[j].y;

                    if((fabs(x1)<=0.01)&&(fabs(y1)<=0.01))
                    {
                        continue;
                    }
                    if((fabs(y1)<=0.01)) 
                    {
                        h=(h>0? h+1:2);
                        curmax=max(curmax,h);
                    } 
                    else if((fabs(x1)<=0.01))  
                    {
                        v=(v>0?v+1:2);
                        curmax=max(curmax,v);
                    }                
                    else
                    {
                        double k=int(y1/x1*10.0)/10.0;//保留一位小数 ****
                        mp[k]=(mp[k]>0? mp[k]+1:2);
                        curmax=max(curmax,mp[k]);
                    }
                   //ROS_INFO("fabs(x1):%.3f,fabs(y1):%.3f,k:%2f,v:%i,h:%i,mp[k]:%i",fabs(x1),fabs(y1),int(y1/x1*100.0)/100.0,v,h,mpk);
                    
                }    
            }
            ret=max(ret,curmax);
        }
        return ret;
    }

double TRIPlannerROS::get_orientation(tf::Stamped<tf::Pose> global_pose,double local_goal_x,double local_goal_y)
{
      double x = global_pose.getOrigin().x(),
             y = global_pose.getOrigin().y();
      double yaw = tf::getYaw(global_pose.getRotation());      
      
      double plan_yaw  = atan2((local_goal_y-y),(local_goal_x-x));  
      double robot_yaw=yaw;
      double orientation,orientation_tmp;
      orientation_tmp=robot_yaw-plan_yaw;
      if(orientation_tmp>3.141592654)
          orientation=orientation_tmp-2*3.141592654;
      else if(orientation_tmp<-3.141592654)
          orientation=orientation_tmp+2*3.141592654;
      else
          orientation=orientation_tmp;
      //ROS_INFO("angle_r:%f angle_p:%f angle_s:%f",robot_yaw/3.141592654*180,plan_yaw/3.141592654*180,orientation/3.141592654*180);
      
      return orientation*180/3.141592654;        
}


 bool TRIPlannerROS::isRobotSafe(tf::Stamped<tf::Pose> global_pose)
 {
   probeGenerator(global_pose);
   double threshold = 220;
   if(!checkLocalPlan(transformed_plan))
   return false;
   
   for(int i=4;i<6;i++)
   {
      if(checkProbeCost[i]>threshold)
          ROS_INFO("cost[%i]:%f",i,checkProbeCost[i]);            
   }
   if((checkProbeCost[4]<threshold)&&(checkProbeCost[5]<threshold))  
   {
        if(recovery_counter!=0)
        {
            recovery_counter_add++;
            if(recovery_counter_add == 5)
            {
                //ROS_INFO("Good to go!");
                return true;
            }
            else 
                return false;
        }
        //ROS_INFO("Good to go!");
        return true;
   }
    else 
        return false;

       return true;
 }
 
/*
 *generate the checking probe and checking points
 */
 void TRIPlannerROS::probeGenerator(tf::Stamped<tf::Pose> global_pose)
 {
   double origin_x = global_pose.getOrigin().getX();
   double origin_y = global_pose.getOrigin().getY();
   double origin_yaw = tf::getYaw(global_pose.getRotation());
   unsigned int mx,my;
   checkProbeCost.clear();
   checkProbe.clear();
   probe_markers.markers.clear();
   int k=0;
   for(int i = 4;i<6 ; i++)
   {
    geometry_msgs::Pose probe_tmp;
    double prob_y = origin_y+sin(origin_yaw)*i/20;
    double prob_x = origin_x+cos(origin_yaw)*i/20;
    probe_tmp.position.x = prob_x;
    probe_tmp.position.y = prob_y;
    probe_tmp.position.z = 0;
    probe_tmp.orientation.x = global_pose.getRotation().x();
    probe_tmp.orientation.y = global_pose.getRotation().y();
    probe_tmp.orientation.z = global_pose.getRotation().z();
    probe_tmp.orientation.w = global_pose.getRotation().w();
    checkProbe.push_back(probe_tmp);
    costmap_->worldToMap(probe_tmp.position.x, probe_tmp.position.y, mx, my);
    checkProbeCost.push_back(costmap_->getCost(mx,my));
    
    
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "prob";
    marker.id = k++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = prob_x;
    marker.pose.position.y = prob_y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = global_pose.getRotation().x();
    marker.pose.orientation.y = global_pose.getRotation().y();
    marker.pose.orientation.z = global_pose.getRotation().z();
    marker.pose.orientation.w = global_pose.getRotation().w();
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    probe_markers.markers.push_back(marker);
   
    }
    probe_pub.publish(probe_markers);
 }

void TRIPlannerROS::setGoal(geometry_msgs::PoseStamped& goal)
{
 plan_goal_pose = goal;
 position_reached_check=false;
 updateStartPose=false;
 ROS_ERROR("x:%f,y:%f",plan_goal_pose.pose.position.x,plan_goal_pose.pose.position.y);
}

bool TRIPlannerROS::isPositionReached(tf::Stamped<tf::Pose> global_pose)
{   
    ROS_ASSERT(plan_goal_pose.header.frame_id!="");
    transformGoalPose(*tf_,plan_goal_pose,goal_pose,"map");
    double goal_x = goal_pose.getOrigin().getX();
    double goal_y = goal_pose.getOrigin().getY();
    
 if(!position_reached_check)
 {    
        //ROS_ERROR("Transformed GOAL x:%f,y:%f",goal_x, goal_y);
    double cal_dis=hypot(goal_x - global_pose.getOrigin().x(), goal_y - global_pose.getOrigin().y());
    if(cal_dis<=xy_goal_tolerance_)
    {
        position_reached_check=true;
        stop_times_ = 5;
        return true;
    }

    return false;
 }

    return true;
}

bool TRIPlannerROS::isGoalReached() {
    tf::Stamped<tf::Pose> global_pose;
    costmap_->getRobotPose(global_pose);
    
    double rot_stopped_vel = 0.0;
    double trans_stopped_vel = 0.1;
    ROS_ASSERT(plan_goal_pose.header.frame_id!="");
    transformGoalPose(*tf_,plan_goal_pose,goal_pose,"map");
    double goal_x = goal_pose.getOrigin().getX();
    double goal_y = goal_pose.getOrigin().getY();
    double cal_dis=hypot(goal_x - global_pose.getOrigin().x(), goal_y - global_pose.getOrigin().y());
    
    double goal_th = tf::getYaw(goal_pose.getRotation());
    double angle = getGoalOrientationAngleDifference(current_pose_, goal_th);
    
     tf::Stamped<tf::Pose> robot_vel;
     odom_helper_.getRobotVel(robot_vel); 
     double robot_vel_x=robot_vel.getOrigin().getX();
     double robot_vel_th=tf::getYaw(robot_vel.getRotation());
    if( cal_dis <= xy_goal_tolerance_) {
        if (fabs(angle) <= yaw_goal_tolerance_){
            if(robot_vel_x <=trans_stopped_vel && robot_vel_th <= trans_stopped_vel){
                stop_times_ = 0;
                return true;
            }
        }
    }
    return false;

}

void TRIPlannerROS::computeVelocityCommandsStopRotate(geometry_msgs::Twist& cmd_vel)
{
  //check to see if the goal orientation has been reached
  double goal_th = tf::getYaw(goal_pose.getRotation());
  double angle = getGoalOrientationAngleDifference(current_pose_, goal_th);
  if (fabs(angle) <= yaw_goal_tolerance_) {
    //set the velocity command to zero
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0; 
    stop_rotate_dir = 0;
  } 
  else {  
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      if(stop_rotate_dir == 0)
      {
       if(angle > 0){
       stop_rotate_dir = 1;
          if(angle > 0.4)
          {
             adjust_w(w_set,0.2,0.5,true);
          }
          else
          {
             w_set=0.2;
          }
          cmd_vel.angular.z = w_set;
       }
       else{
       stop_rotate_dir = -1;
          if(angle < -0.4)
          {
             adjust_w(w_set,-0.5,-0.2,false);
          }
          else
          {
              w_set=-0.2;
          }
          cmd_vel.angular.z = w_set;  
       }
      }
      if(stop_rotate_dir == 1)
      {
          if(angle > 0.4)
          {
             adjust_w(w_set,0.2,0.5,true);
          }
          else
          {
             w_set=0.2;
          }
          cmd_vel.angular.z = w_set;
      }
      if(stop_rotate_dir == -1)
      {
          if(angle < -0.4)
          {
             adjust_w(w_set,-0.5,-0.2,false);
          }
          else
          {
              w_set=-0.2;
          }
          cmd_vel.angular.z = w_set;   
      }
  }
}


bool TRIPlannerROS::isOrientationAjusted(tf::Stamped<tf::Pose> global_pose) {
   double local_goal_x = transformed_plan.back().pose.position.x;
   double local_goal_y = transformed_plan.back().pose.position.y;
   
   for(std::vector<geometry_msgs::PoseStamped>::const_iterator iter = transformed_plan.begin(); iter!=transformed_plan.end(); iter++)
  {
   if((global_pose.getOrigin().x()-(*iter).pose.position.x)*(global_pose.getOrigin().x()-(*iter).pose.position.x)+
   (global_pose.getOrigin().y()-(*iter).pose.position.y)*(global_pose.getOrigin().y()-(*iter).pose.position.y)>0.25)//1.3
   {
    local_goal_x = (*iter).pose.position.x;
    local_goal_y = (*iter).pose.position.y;
    break;
   }
  } 
 
  double plan_yaw  = atan2((local_goal_y-global_pose.getOrigin().y()),(local_goal_x-global_pose.getOrigin().x()));  
  double robot_yaw=tf::getYaw(current_pose_.getRotation());
  double start_orientation_tmp;
  start_orientation_tmp = robot_yaw - plan_yaw;
  if(start_orientation_tmp>3.141592654) 
      start_orientation = start_orientation_tmp - 2*3.141592654;
  else if(start_orientation_tmp<-3.141592654) 
      start_orientation = start_orientation_tmp + 2*3.141592654;
  else 
      start_orientation = start_orientation_tmp;


    //随速度变化的orientation_tolerance
  orientation_tolerance = std::max(0.2,3*vel_set_+0.1);
  orientation_tolerance = std::min(orientation_tolerance,1.3);  
  if (fabs(start_orientation) > orientation_tolerance) {
    return true; 
  }
  else
    return false;
}

  
  bool TRIPlannerROS::getTransformPlan(tf::Stamped<tf::Pose>& current_pose,const std::vector<geometry_msgs::PoseStamped>& global_plan, std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
  if(gl_plan_vector.size()==0)
      return false;
    transformPlan(global_plan,transformed_plan);
    return true; 
    }
};
