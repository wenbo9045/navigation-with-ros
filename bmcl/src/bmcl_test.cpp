#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "ros/ros.h"
#include <vector>
#include <iostream>
#include "tf/tf.h"
using namespace std;


struct pose_sample
{
 double x;
 double y;
 double th;
};


int main(int argc, char** argv)
{
 ros::init(argc,argv,"bmcl_test");
 ros::NodeHandle n_;
 ros::Publisher pub = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
 geometry_msgs::PoseWithCovarianceStamped sample_pose; 
 ros::Rate rate(5);
 sample_pose.header.seq = 1;
 sample_pose.header.stamp = ros::Time::now();
 sample_pose.header.frame_id = "/map";
 sample_pose.pose.covariance = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};
  vector<pose_sample> set;

  double x_min = -0.125;
  double x_max = 0.125;
  double y_min = -0.125;
  double y_max = 0.125;
  double th_min = -0.261799;
  double th_max = 0.261799;
  double dist_delta = 0.01;
  double ang_delta = 0.0324;
  for(double x_input = x_min; x_input <= x_max; x_input += dist_delta)
  {
   for(double th_input = th_min; th_input <= th_max; th_input += ang_delta)
   {
    pose_sample ps;
    ps.x = x_input;
    ps.th = th_input;
    ps.y = y_min;
    set.push_back(ps);
    ps.y = y_max;
    set.push_back(ps);
   }
  }
  
  for(double y_input = y_min; y_input <= y_max; y_input += dist_delta)
  {
   for(double th_input = th_min; th_input <= th_max; th_input += ang_delta)
   {
    pose_sample ps;
    ps.y = y_input;
    ps.th = th_input;
    ps.x = x_min;
    set.push_back(ps);
    ps.x = x_max;
    set.push_back(ps);
   }
  }
  cout<<"number: "<<set.size()<<endl;
  int size = set.size();
  while(size--)
  {
   tf::Transform transform1(tf::createQuaternionFromYaw(set[size].th),tf::Vector3(set[size].x,set[size].y,0));
   sample_pose.pose.pose.position.x = transform1.getOrigin().x();
   sample_pose.pose.pose.position.y = transform1.getOrigin().y();
   sample_pose.pose.pose.orientation.z = transform1.getRotation().z();
   sample_pose.pose.pose.orientation.w = transform1.getRotation().w(); 
   pub.publish(sample_pose);  
   rate.sleep();
  }
 return 0;
}
