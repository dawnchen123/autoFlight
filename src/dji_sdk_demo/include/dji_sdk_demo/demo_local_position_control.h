/** @file demo_local_position_control.h
 *  @version 3.3
 *  @date September, 2017
 *
 *  @brief
 *  demo sample of how to use local position control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef PROJECT_DEMO_LOCAL_POSITION_CONTROL_H
#define PROJECT_DEMO_LOCAL_POSITION_CONTROL_H

#endif //PROJECT_DEMO_LOCAL_POSITION_CONTROL_H

#include <dji_sdk/SetLocalPosRef.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>

#include <nav_msgs/Odometry.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h> 
#include <sensor_msgs/image_encodings.h> 
#include <image_transport/image_transport.h> 
#include <opencv2/opencv.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include <stdio.h> 
#include <math.h> 
#include <vector> 
#include <iostream>
#include <chrono>
bool set_local_position();

float target_offset_x;
float target_offset_y;
float target_offset_z;
float target_yaw;
int target_set_state = 0;

void setTarget(float x, float y, float z, float yaw)
{
  target_offset_x = x;
  target_offset_y = y;
  target_offset_z = z;
  target_yaw      = yaw;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);

bool takeoff_land(int task);

bool landing_land(int task);

bool obtain_control();

bool is_M100();

bool monitoredTakeoff();

bool M100monitoredTakeoff();

bool M100monitoredLand();


void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd);
class PidController
{
private:
	float set_point_;
	float kProportional_;
	float kIntegral_;
	float kDerivative_;
	float previous_error_;
	bool previous_set;
	float sum_;
	std::chrono::time_point<std::chrono::system_clock> prev_time_;
public:
	//set desired point.
	void setPoint(float target, float kProportional, float kIntegral, float kDerivative) {
		set_point_ = target;
		kProportional_ = kProportional;
		kIntegral_ = kIntegral;
		kDerivative_ = kDerivative;
		prev_time_ = std::chrono::system_clock::now();
		previous_error_ = 0;
		sum_ = 0;
		previous_set = false;
	}
	float control(float processVariable) {
		auto t = std::chrono::system_clock::now();
		auto diff = std::chrono::system_clock::now() - prev_time_;

		float error = set_point_ - processVariable;
		if (!previous_set)
		{
			previous_set = true;
			previous_error_ = error;
			return 0;
		}

		float dt = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(diff).count()) * 0.000001f;
		dt = fmax(dt, 0.01f);
		float proportionalGain = 0;
		float derivativeGain = 0;
		float integralGain = 0;

		if (kProportional_ != 0) {
			proportionalGain = error * kProportional_;
		}
		if (kDerivative_ != 0) {
			float derivative = (error - previous_error_) / dt;
			derivativeGain = derivative * kDerivative_;
		}
		if (kIntegral_ != 0) {
			sum_ += error * dt;
			integralGain = sum_ * kIntegral_;
		}

		previous_error_ = error;
		prev_time_ = t;

		return proportionalGain + derivativeGain + integralGain;
	}
};

// void imageCalllback(const sensor_msgs::ImageConstPtr& msg); 
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void heightCallback(const std_msgs::Float32::ConstPtr& msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);