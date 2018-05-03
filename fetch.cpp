#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdlib.h>

#include <vector>
#include <boost/array.hpp>
#include <cmath>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Twist.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>



// Declare class variables, messages, publishers

ros::Subscriber point_cloud_subscriber;
ros::Publisher vel_cmd_pub;
ros::Publisher pcl_pub;
bool ranOnce = false;


using std::vector;
using Eigen::Vector3f;
using namespace std;


// Callbacks


Vector3f rgb_find_center(pcl::PointCloud<pcl::PointXYZRGB> cloud){

 
//////////////////////////////////////////////////////////////////


   // estimated RGB for a tennis ball - https://www.rapidtables.com/web/color/RGB_Color.html
   float model_r = 130;
   float model_g = 255;
   float model_b = 0;

   float threshold = 150; // threshold distance to be included in RGB filtering
   float ball_diameter = 0.1; // tennis ball diameter is about 10 cm

 
///////////////////////////////////////////////////////////////////


   Vector3f rgb_ball(model_r, model_g, model_b);
   Vector3f rgb_point(0,0,0);

   float dist = 0;

   // get points within RGB threshold
   pcl::PointIndices indices;
   for(unsigned int i = 0; i < cloud.points.size(); i++)
     {
        rgb_point = Vector3f(cloud.points[i].r, cloud.points[i].g, cloud.points[i].b);
	dist = (rgb_ball-rgb_point).norm();
	if(dist < threshold)
           indices.indices.push_back(i);
     }

   // variables for picking random point
   int num_rgb_match = indices.indices.size();
   int iter_count = 0;
   int rand_pick = 0;
   float p = 0;
   pcl::PointIndices indices2;
   Vector3f rand_point;
   Vector3f compare_point;

   // pick random points from RGB filtered cloud until we find a good model (RANSAC)
   while(p < 0.3 && iter_count < 20){
	indices2.indices.clear();

	rand_pick = rand() % num_rgb_match;
	rand_point(0) = cloud.points[indices.indices[rand_pick]].x;
	rand_point(1) = cloud.points[indices.indices[rand_pick]].y;
	rand_point(2) = cloud.points[indices.indices[rand_pick]].z;

	for(int i = 0; i < num_rgb_match; i++){
		compare_point(0) = cloud.points[indices.indices[i]].x;
		compare_point(1) = cloud.points[indices.indices[i]].y;
		compare_point(2) = cloud.points[indices.indices[i]].z;

		dist = (rand_point-compare_point).norm();
		if(dist < ball_diameter)
			indices2.indices.push_back(indices.indices[i]);
		
	}

	p = (float)indices2.indices.size() / (float)num_rgb_match;

	iter_count++;
	if(iter_count==20){
		ROS_INFO("Ran too many iterations, terminating loop");
	}
   }

   int num_ball_indices = indices2.indices.size();
   ROS_INFO("Num ball indices: %d", num_ball_indices);
   // end if we got too few points
   if (num_ball_indices < 5)
	return Vector3f(0,0,0);
   

   // mark selected points as bright green
   for (unsigned int m=0; m<indices2.indices.size(); ++m)
     {
       cloud.points[indices2.indices[m]].r = 0;
       cloud.points[indices2.indices[m]].g = 255;
       cloud.points[indices2.indices[m]].b = 0;

     }
     pcl_pub.publish(cloud);

   Vector3f center = Vector3f(0,0,0);
   Vector3f add_vector = Vector3f(0,0,0);
   for(int i = 0; i < num_ball_indices; i++){
	add_vector(0) = cloud.points[indices2.indices[i]].x;
	add_vector(1) = cloud.points[indices2.indices[i]].y;
	add_vector(2) = cloud.points[indices2.indices[i]].z;

	center += add_vector;

   }

   center = center/num_ball_indices;

   return center;

}




void execute_movements(float x_ball, float y_ball){


  float lin_speed = 0.1;
  float ang_speed = 0.1;
  if(y_ball < 0){
	ang_speed = -ang_speed;
  }
  float d = sqrt(exp2(x_ball) + exp2(y_ball));
  float theta = atan(y_ball/x_ball);

  float move_duration = abs(d/lin_speed); // +0.1?
  float turn_duration = abs(theta/ang_speed); // +0.1?
  float complete_turn_duration = abs(3.141592/ang_speed);
  

  ros::Rate rate(2);

  if(!ranOnce){
    geometry_msgs::Twist turn1;
    geometry_msgs::Twist move1;
    geometry_msgs::Twist turn2;
    geometry_msgs::Twist move2;
    geometry_msgs::Twist stop;

    turn1.linear.x = 0;
    turn1.linear.y = 0;
    turn1.linear.z = 0;

    turn1.angular.x = 0;
    turn1.angular.y = 0;
    turn1.angular.z = ang_speed;


    move1.linear.x = lin_speed;
    move1.linear.y = 0;
    move1.linear.z = 0;

    move1.angular.x = 0;
    move1.angular.y = 0;
    move1.angular.z = 0;


    stop.linear.x = 0;
    stop.linear.y = 0;
    stop.linear.z = 0;

    stop.angular.x = 0;
    stop.angular.y = 0;
    stop.angular.z = 0;

    //turn by theta for 1 second
    ros::Time start = ros::Time::now();
    ROS_INFO("beginning orientation");
    while(ros::Time::now() - start < ros::Duration(turn_duration)){
      vel_cmd_pub.publish(turn1);
    }

    vel_cmd_pub.publish(stop);
    rate.sleep();
    ROS_INFO("beginning forward");
    start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(move_duration)){
      vel_cmd_pub.publish(move1);
    }

    vel_cmd_pub.publish(stop);
    rate.sleep();
    ROS_INFO("beginning 180");
    start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(complete_turn_duration)){
      vel_cmd_pub.publish(turn1);
    }
    

    vel_cmd_pub.publish(stop);
    rate.sleep();
    ROS_INFO("beginning return");
    start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(move_duration)){
      vel_cmd_pub.publish(move1);
    }

    vel_cmd_pub.publish(stop);
  }

  
  ranOnce = true;


}


void PointCloudCallback(pcl::PointCloud<pcl::PointXYZRGB> cloud){

	Vector3f center = rgb_find_center(cloud);
	ROS_INFO("Center: x: %f, y: %f, z: %f", center(0), center(1), center(2));

	
	float rob_x = center(2);
	float rob_y = -center(0);
	// translate
	if(center.norm() > 0.1){
	   execute_movements(rob_x, rob_y);
	}
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "fetch");
  ros::NodeHandle n;

  point_cloud_subscriber = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, PointCloudCallback);

  pcl_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/pcl_test", 1);
  vel_cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  ros::spin();

  return 0;
}
