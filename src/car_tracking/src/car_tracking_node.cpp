#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include "geometry_msgs/PoseStamped.h"
#include "bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h"

using namespace std;
using namespace cv;
#define FX 537.292878
#define FY 527.000348
#define CX 427.331854
#define CY 240.226888
#define PI 3.14159265
static const string OPENCV_WINDOW = "Image window";
Point lastcarPosition = Point(0, 0);
int counter = 0;

bool distance(Mat3b img, double roll, double pitch, double yaw, double lens, double width, double z, double distance[3]);
bool findRedCenter(Mat3b img, Point &carPosition);
void camera(double matriz[4][4]);
void relativePosition(double position[3], Point carPosition, double roll, double pitch, double yaw, double lens, double width, double z);
void findPosition(double matriz[4][4], double cameraMatriz[4][4], Point p, double z, double position[3]);
void lensToDroneCenter(double matriz[4][4], double roll, double pitch, double yaw, double lens, double width);
void pitchRotation(double matriz[4][4], double pitch);
void rollRotation(double matriz[4][4], double roll);
void moveToLens(double matriz[4][4], double width);
void lensRotation(double matriz[4][4], double lens);
void multMatriz(double matriz[4][4], double mult[4][4], double answer[4][4]);
void yawRotation(double matriz[4][4], double yaw);

class CarTracking
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber altitude_sub_;
  ros::Publisher pose_pub_;
  image_transport::Publisher image_pub_;

private:
  nav_msgs::Odometry odometry_;
  double altitude_;

public:
  CarTracking()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/bebop/image_raw", 1,
      &CarTracking::imageCallback, this);
    odometry_sub_ = nh_.subscribe("/bebop/odom", 1,
      &CarTracking::OdometryCallback, this);
    altitude_sub_ = nh_.subscribe("/bebop/states/ardrone3/PilotingState/AltitudeChanged", 1,
      &CarTracking::AltitudeCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("car_pose", 1000);
    image_pub_ = it_.advertise("/bebop/image_analyzed", 1);
  }

  ~CarTracking()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void AltitudeCallback(const bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& altitude_msg) {
    altitude_ = altitude_msg->altitude;
/*    cout << "Altitudeeeee!!!!!!" <<endl;
    cout << altitude_ << endl;*/
      
  }

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    odometry_.pose.pose.position = odometry_msg->pose.pose.position;
    odometry_.pose.pose.orientation = odometry_msg->pose.pose.orientation;
    odometry_.twist.twist.linear = odometry_msg->twist.twist.linear;
    odometry_.twist.twist.angular = odometry_msg->twist.twist.angular;
      
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    double distance_control[3];
    geometry_msgs::PoseStamped car_pose;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    tf::Quaternion q(odometry_.pose.pose.orientation.x, odometry_.pose.pose.orientation.y, odometry_.pose.pose.orientation.z, odometry_.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    if (distance(cv_ptr->image, roll, pitch, yaw, 2.3552 , 0.2, altitude_, distance_control)) {
      car_pose.header.stamp = msg->header.stamp;
      car_pose.header.frame_id = "base_link";
      car_pose.pose.position.x = distance_control[0];
      car_pose.pose.position.y = distance_control[1];
      car_pose.pose.position.z = distance_control[2];
      pose_pub_.publish(car_pose);
      image_pub_.publish(cv_ptr->toImageMsg());
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_tracking");
  CarTracking ct;
  ros::spin();
  return 0;
}



bool distance(Mat3b img, double roll, double pitch, double yaw, double lens, double width, double z, double position[3]){
  double ideal[3], distance[3];
  Point carPosition;
  bool foundCar = findRedCenter(img, carPosition);
  /*cout << carPosition <<endl;*/
  relativePosition(position, carPosition, roll, pitch, yaw, lens, width, z);
  relativePosition(ideal, Point(CX, CY), roll, pitch, yaw, lens, width, z);

  position[0] = -ideal[0] + position[0];
  position[1] = -ideal[1] + position[1];
  position[2] = -ideal[2] + position[2];
  return foundCar;
}

bool findRedCenter(Mat3b img, Point &carPosition){
  // Converte a imagem RGB para HSV
  Mat3b hsv;
    cvtColor(img, hsv, COLOR_BGR2HSV);
  
  //Já que o vermelho esta na transição da escala de hsv,
  // precisamos realizar duas mascaras.
  //Para fazer o ajuste fino da cor, alterar os valores abaixo.
  Scalar red_low1 = Scalar(0, 120, 100);
  Scalar red_high1 = Scalar(10, 255, 255);
  Scalar red_low2 = Scalar(170, 120, 100);
  Scalar red_high2 = Scalar(180, 255, 255);
  
  //Constroi a mascara da imagem entre os valores especificados
    Mat1b mask1, mask2;
    inRange(hsv, red_low1, red_high1, mask1);
    inRange(hsv, red_low2, red_high2, mask2);
    Mat1b mask = mask1 | mask2; 
  
  vector<vector<Point> > contours;
  int largest_area=0;
  int largest_contour_index=0;
  findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));


  if  (contours.size() > 0) {
    //cout << contours.size() <<endl;
    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
      {
       double a=contourArea( contours[i],false);  //  Find the area of contour
       if(a>largest_area){
       largest_area=a;
       largest_contour_index=i;                //Store the index of largest contour
       }
  
      }
    cout << largest_area <<endl;
    if (largest_area > 30 && largest_area < 100000){
    }
    else{
      return false;
    }
    Moments mu =  moments(contours[largest_contour_index], false );
    //cout << "m10: " << mu.m10 << endl;
    //cout << "m01: " << mu.m01 << endl;
    //cout << "m00: " << mu.m00 << endl;
    /*cout << mu.m10/mu.m00 << endl;
    cout << mu.m01/mu.m00 << endl;*/
    carPosition =  Point(mu.m10/mu.m00, mu.m01/mu.m00);

    if (lastcarPosition.x == carPosition.x && lastcarPosition.y==carPosition.y){
      counter++;
      cout << "SAME IMAGE!!!!!" << endl;
      if (counter >= 15){
        ROS_ERROR("MATANDO NO BEBOP DRIVER!!!!!!!!!!");
        //system("rosnode kill /bebop/bebop_driver");
        return false;
      }
    }
    else{
      counter = 0;
    }

    lastcarPosition.x = carPosition.x;
    lastcarPosition.y = carPosition.y;
    circle( img, carPosition, 5, Scalar(0,255,0), 3);
    vector<vector<Point> > contours2;
    contours2.push_back(contours[largest_contour_index]);
    drawContours(img, contours2, -1, Scalar(0,255,0), 3);
    return true;
  }
  else{
    return false;
  }
  
}

void relativePosition(double position[3], Point carPosition, double roll, double pitch, double yaw, double lens, double width, double z){
  double droneMatriz[4][4], cameraMatriz[4][4];
  
  camera(cameraMatriz);
  lensToDroneCenter(droneMatriz, roll, pitch, yaw, lens, width);
  
  findPosition(droneMatriz, cameraMatriz, carPosition, z, position);
} 

void camera(double matriz[4][4]){
  for (int i = 0; i<4; i++){
    for (int j = 0; j<4; j++){
      matriz[i][j] = 0.0;
    }
  }
  matriz[0][0] = FX;
  matriz[1][1] = FY;
  matriz[0][2] = CX;
  matriz[1][2] = CY;
  matriz[2][2] = 1.0;
  matriz[3][3] = 1.0;
}

void findPosition(double matriz[4][4], double cameraMatriz[4][4], Point p, double z, double position[3]){
  double m[4][4];
  multMatriz(cameraMatriz, matriz, m);  
  
  position[0] = ((m[1][2]-p.y*m[2][2])*z+m[1][3]-p.y*m[2][3])*(m[0][1]-p.x*m[2][1]);
  position[0] += (p.y*m[2][1]-m[1][1])*((m[0][2]-p.x*m[2][2])*z+m[0][3]-m[2][3]*p.x);
  position[0] = position[0]/((p.y*m[2][1]-m[1][1])*(p.x*m[2][0]-m[0][0])+(m[1][0]-p.y*m[2][0])*(p.x*m[2][1]-m[0][1]));
  
  position[1] = ((m[1][0]-p.y*m[2][0])*position[0]+(m[1][2]-p.y*m[2][2])*z+m[1][3]-p.y*m[2][3])/(p.y*m[2][1]-m[1][1]);
  
  position[2] = z;
}

void lensToDroneCenter(double matriz[4][4], double roll, double pitch, double yaw, double lens, double width){
  double pitchMatriz[4][4], rollMatriz[4][4], moveMatriz[4][4], lensMatriz[4][4], aux[4][4], yawMatriz[4][4], aux2[4][4], aux3[4][4];
  pitchRotation(pitchMatriz, pitch);
  rollRotation(rollMatriz, roll);
  moveToLens(moveMatriz, width);
  rollRotation(lensMatriz, lens);
  yawRotation(yawMatriz, -PI/2);
  multMatriz(lensMatriz, yawMatriz, aux);
  multMatriz(aux, moveMatriz, aux2);
  multMatriz(aux2, rollMatriz, aux3);
  multMatriz(aux3, pitchMatriz, matriz);
}

void pitchRotation(double matriz[4][4], double pitch){
  for (int i = 0; i<4; i++){
    for (int j = 0; j<4; j++){
      matriz[i][j] = 0.0;
    }
  }
  matriz[0][0] = cos(-pitch);
  matriz[0][2] = -sin(-pitch);
  matriz[1][1] = 1.0;
  matriz[2][2] = cos(-pitch);
  matriz[2][0] = sin(-pitch);
  matriz[3][3] = 1.0;  
} 

void rollRotation(double matriz[4][4], double roll){
  for (int i = 0; i<4; i++){
    for (int j = 0; j<4; j++){
      matriz[i][j] = 0.0;
    }
  }
  matriz[0][0] = 1.0;
  matriz[1][1] = cos(-roll);
  matriz[1][2] = sin(-roll);
  matriz[2][1] = -sin(-roll);
  matriz[2][2] = cos(-roll);
  matriz[3][3] = 1.0;
}

void yawRotation(double matriz[4][4], double yaw){
  for (int i = 0; i<4; i++){
    for (int j = 0; j<4; j++){
      matriz[i][j] = 0.0;
    }
  }
  matriz[0][0] = cos(-yaw);
  matriz[0][1] = sin(-yaw);
  matriz[1][0] = -sin(-yaw);
  matriz[1][1] = cos(-yaw);
  matriz[2][2] = 1.0;
  matriz[3][3] = 1.0;
}

void moveToLens(double matriz[4][4], double width){
  for (int i = 0; i<4; i++){
    for (int j = 0; j<4; j++){
      matriz[i][j] = 0.0;
    }
  }
  
  matriz[0][0] = 1.0;
  matriz[1][1] = 1.0;
  matriz[2][2] = 1.0;
  matriz[0][3] = width/2;
  matriz[3][3] = 1.0;
} 
  
void multMatriz(double matriz[4][4], double mult[4][4], double answer[4][4]){
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      answer[i][j] = 0;
      for(int k = 0; k<4; k++){
        answer[i][j] += matriz[i][k]*mult[k][j];
      }
    }
  }
}