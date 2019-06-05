#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream> 
#include <string>
using namespace std;
FILE *fp;
int main(int argc,char** argv)
{
    ros::init(argc,argv,"data_test");
    ros::NodeHandle n;
    string file_name="/home/zmy/info_x.dat";
//    ros::spin();
    fp=fopen(file_name.c_str(),"w+");
    fprintf(fp,"%i",12);
    fprintf(fp,"%s","\n");    
    fclose(fp);
    return 0;
}

