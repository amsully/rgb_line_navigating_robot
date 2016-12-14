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
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

using Eigen::Vector2f;
using geometry_msgs::Point;
using std::fabs;
using std::max;
using std::atan2;
using std::cout;
using std::vector;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

// Setting Robot Vars.
static const char WINDOW[] = "Image Window";

float velocity_angular, velocity_linear, new_velocity_angular, new_velocity_linear;
float d_angular, d_linear, d_t = 0.5;
float horizontalCount;

// Initial Robot Vars.
double linear_v = 0.05;
double angular_v = 0.05;
double linear_scale = 1.0;
double angular_scale = 1.0; // for decreasing/increasing speed
double left_threshold = 150;
double right_threshold = 450;

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

// Initialize all markers.
void InitMarkers() {
  vertices_marker_.header.frame_id = "map";
  vertices_marker_.id = 1;
  vertices_marker_.type = Marker::POINTS;
  vertices_marker_.action = Marker::MODIFY;
  vertices_marker_.scale.x = 0.2;
  vertices_marker_.scale.y = 0.2;
  vertices_marker_.color.a = 1.0;
  vertices_marker_.color.r = 0.0;
  vertices_marker_.color.g = 0.0;
  vertices_marker_.color.b = 1.0;


  qrand_marker_.header.frame_id = "map";
  qrand_marker_.id = 2;
  qrand_marker_.type = Marker::POINTS;
  qrand_marker_.action = Marker::MODIFY;
  qrand_marker_.scale.x = 0.2;
  qrand_marker_.scale.y = 0.2;
  qrand_marker_.color.a = 1.0;
  qrand_marker_.color.r = 1.0;
  qrand_marker_.color.g = 0.0;
  qrand_marker_.color.b = 0.0;

  edges_marker_.header.frame_id = "map";
  edges_marker_.id = 3;
  edges_marker_.type = Marker::LINE_LIST;
  edges_marker_.action = Marker::MODIFY;
  edges_marker_.scale.x = 0.05;
  edges_marker_.scale.y = 0.05;
  edges_marker_.color.a = 1.0;
  edges_marker_.color.r = 0.0;
  edges_marker_.color.g = 1.0;
  edges_marker_.color.b = 0.0;

  map_marker_.header.frame_id = "map";
  map_marker_.id = 4;
  map_marker_.type = Marker::LINE_LIST;
  map_marker_.action = Marker::MODIFY;
  map_marker_.scale.x = 0.05;
  map_marker_.scale.y = 0.05;
  map_marker_.color.a = 1.0;
  map_marker_.color.r = 1.0;
  map_marker_.color.g = 1.0;
  map_marker_.color.b = 1.0;

  plan_marker_.header.frame_id = "map";
  plan_marker_.id = 5;
  plan_marker_.type = Marker::LINE_LIST;
  plan_marker_.action = Marker::MODIFY;
  plan_marker_.scale.x = 0.05;
  plan_marker_.scale.y = 0.05;
  plan_marker_.color.a = 1.0;
  plan_marker_.color.r = 1.0;
  plan_marker_.color.g = 0.0;
  plan_marker_.color.b = 0.0;
}

void processImage(const sensor_msgs::ImageConstPtr& raw_image){

	//cv_bridge::CvBridge bridge = cv_bridge::CvBridge();
	//IplImage* img = bridge.imgMsgToCv(msg,"rgb8"); 
	// IplImage* img = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);	

//	cv_bridge::CvImagePtr cv_ptr;
//	cv_ptr = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);	
//	IplImage img = cv_ptr->image;

//	geometry_msgs::Twist velMsg;

//	CvMemStorage* storage = cvCreateMemStorage(0);
	
	cout << raw_image;
	
}

void AstraRGBImageCallback(const sensor_msgs::Image& raw_image){
  cout << "Hello World";
}

void USBCamRGBImageCallback(const sensor_msgs::ImageConstPtr& raw_image){

  processImage(raw_image);
  cout << "Hello World";
}


int main(int argc, char **argv) {
  InitMarkers();

  ros::init(argc, argv, "rgb_line_navigating_robot");
  ros::NodeHandle n;

  cmd_publisher_ = n.advertise <geometry_msgs::Twist>("cmd_vel",1);


  // Astra Launch
  ros::Subscriber astra_rgb_raw_subscriber = n.subscribe("/camera/rgb/image_raw", 1, AstraRGBImageCallback);

  // usb_cam
  ros::Subscriber ubs_cam_rgb_raw_subscriber = n.subscribe("/usb_cam/image_raw", 1, USBCamRGBImageCallback);

  ros::spin();
  return 0;
}
