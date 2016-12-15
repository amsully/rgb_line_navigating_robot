#include <algorithm>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h> 
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
// #include <cvaux.h>
#include <opencv/cvaux.h>
#include <math.h>
// #include <cxcore.h>
#include <opencv/cxcore.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

using Eigen::Vector2f;
using Eigen::Vector3f;
using geometry_msgs::Point;
using std::fabs;
using std::max;
using std::atan2;
using std::cout;
using std::vector;
using std::min;
using std::max;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

// TESTING PARAMS
bool show_light_reading =false;

// SETTING PARAMS
bool run_simpleProcessImage = false;
bool run_binaryProcessImage = true;

// Global params
float velocity_angular, velocity_linear;
int currentLightValue;


// Initial Robot Vars.
double linear_v = 0.05;
double angular_v = 0.05;
double max_linear_v = 0.3;
double max_angular_v = 0.5;


// Image split
// int left_pix = 930;
// int right_pix = 990;

int right_pix = 1160;
int left_pix = 760;

// SET BASED ON ENVIRONMENT
int RANGE = 3;
int GREEN = 51;

int angularAdjustments = 0;
int maxAngularAdjustments = 50;
bool handlingEvent = false;

// Publisher for marker messages.
ros::Publisher markers_publisher_;
ros::Publisher cmd_publisher_;

// Markers for visualization.
Marker vertices_marker_;
Marker qrand_marker_;
Marker edges_marker_;
Marker map_marker_;
Marker plan_marker_;

float delta_q = 0.5;

vector<Point> start_map;
vector<Point> end_map;

// Return a random value between min and max.
float RandomValue(const float min, const float max) {
  const float scale = max - min;
  const float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  return (min + r * scale);
}

// Helper function to convert a 2D vector into a ros 3D point.
Point VectorToPoint(const Vector2f& v) {
  Point point;
  point.x = v.x();
  point.y = v.y();
  point.z = 0;
  return point;
}

// Helper function to convert ROS Point32 to Eigen Vectors.
Vector2f ConvertPointToVector(Point point) {
  return Vector2f(point.x, point.y);
}

// Helper function to visualize a point.
void DrawPoint(const Vector2f& p, Marker* marker) {
  marker->points.push_back(VectorToPoint(p));
}

// Helper function to visualize an edge.
void DrawLine(const Vector2f& p1,
              const Vector2f& p2,
              Marker* marker) {
  marker->points.push_back(VectorToPoint(p1));
  marker->points.push_back(VectorToPoint(p2));
}

void safeAngularDecrement(){
  velocity_angular =  max(velocity_angular - angular_v, -max_angular_v) ;
}

void safeAngularIncrement(){
  velocity_angular = min(velocity_angular + angular_v, max_angular_v);
}

void safeLinearDecrement(){
  velocity_linear =  max(velocity_linear - linear_v, -max_linear_v) ;
}

void safeLinearIncrement(){
  velocity_linear = min(velocity_linear + linear_v, max_linear_v);
}

void publishTwist(){
  geometry_msgs::Vector3 angular;
  geometry_msgs::Vector3 linear;
  angular.z = velocity_angular;
  angular.y = 0;
  angular.x = 0;
  linear.x = velocity_linear;
  linear.y = 0; 
  linear.z = 0;

  geometry_msgs::Twist new_twist = geometry_msgs::Twist();
  new_twist.angular = angular;
  new_twist.linear = linear;

  cmd_publisher_.publish(new_twist);
}

void adjustAngularVelocity(){
  if(velocity_angular > 0){
     safeAngularDecrement();
  }else{
     safeAngularIncrement();
  }
  angularAdjustments++;
}

// Search for line event (angular velocity only).
void searchForLine(){

	if( !(fabs(currentLightValue - GREEN) < RANGE) ){
		cout << "search: "<< currentLightValue << "\n";
		velocity_angular = min(velocity_angular + angular_v, max_angular_v);	
 	}else{
		velocity_angular = 0;
		angularAdjustments = 0;
	}
}

// The default behavior of the robot when image stream is being posted.
void generalMovement(){
  // if(fabs(currentLightValue - GREEN) < RANGE ){
  //   // I think we are good., continue?

  // // If Robot has not seen line for specified time. Search for it.
  // }else 
  if(angularAdjustments > maxAngularAdjustments){
    searchForLine();
  }else{
    adjustAngularVelocity();
  }

  // Publishes velocity_angular and velocity_linear.
	cout << "pub " <<  velocity_angular << "\n";
  publishTwist();

}

void computeCommands(){
	  generalMovement();
}

void simpleProcessImage(const sensor_msgs::Image& raw_image){
	
  int data_size = raw_image.data.size();
  int curr_val = 0;
  int sum = 0;

  for(size_t i = 0; i < raw_image.data.size(); ++i){
    curr_val = raw_image.data[i];
    sum += curr_val;
  }

 	currentLightValue = sum/(data_size);
	cout << "light:" << currentLightValue << "\n";

  if(!show_light_reading){
  	computeCommands();
  }else{
    cout << "DEBUG: show_light_reading \n";
  }

  // sleep(1);
}

void computeBinaryCommands(int leftAvg, int rightAvg){
  velocity_linear = 0.07;

  if(fabs(leftAvg - rightAvg) < 9){
    velocity_angular = 0;
    safeLinearIncrement();
  }
  if(leftAvg > rightAvg){
    safeAngularDecrement(); // max(velocity_angular - angular_v, -max_angular_v) ;
    safeLinearDecrement();

  }else{
    safeAngularIncrement(); // min(velocity_angular + angular_v, max_angular_v);
    safeLinearDecrement();
  }

  publishTwist();
}

void binaryProcessImage(const sensor_msgs::Image& raw_image){

  int data_size = raw_image.data.size();
  int curr_val;
  int rightSum = 0;
  int leftSum = 0;
  int step = raw_image.step;
  for(size_t i = 0; i < raw_image.data.size(); ++i){
    curr_val = raw_image.data[i];

    if(i%step < step/2){
      leftSum+=curr_val;
    }else{
      rightSum+=curr_val;
    }

  }

  int rightAvg = rightSum/(data_size/2);
  int leftAvg = leftSum/(data_size/2);



  cout << "binary light:" << leftAvg << " " << rightAvg << "\n";

  if(!show_light_reading){
    computeBinaryCommands(leftAvg, rightAvg);
  }else{
    cout << "DEBUG: show_light_reading \n";
  }

  // sleep(1);
}

void AstraRGBImageCallback(const sensor_msgs::Image& raw_image){
  cout << "Hello World";
}

void USBCamRGBImageCallback(const sensor_msgs::Image& raw_image){

  if(run_simpleProcessImage){
    simpleProcessImage(raw_image);
  }else if(run_binaryProcessImage){
    binaryProcessImage(raw_image);
  }
}


int main(int argc, char **argv) {

  ros::init(argc, argv, "rgb_line_navigating_robot");
  ros::NodeHandle n;

  cmd_publisher_ = n.advertise <geometry_msgs::Twist>("/mobile_base/commands/velocity",1);


  // Astra Launch
  ros::Subscriber astra_rgb_raw_subscriber = n.subscribe("/camera/rgb/image_raw", 1, AstraRGBImageCallback);

  // usb_cam
  ros::Subscriber ubs_cam_rgb_raw_subscriber = n.subscribe("/usb_cam/image_raw", 1, USBCamRGBImageCallback);

  ros::spin();
  return 0;
}
