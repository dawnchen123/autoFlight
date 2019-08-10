/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date September, 2017
 *
 *  @brief
 *  demo sample of how to use Local position control
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_sdk_demo/demo_local_position_control.h"
#include "dji_sdk/dji_sdk.h"
//#include <dji_sdk/dji_control.h>



ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlFlightPub;

uint8_t flag;
// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
uint8_t current_gps_health = 0;
int num_targets = 0;

geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix current_gps_position;
sensor_msgs::Joy controlVelYawRate;

int circle_x, circle_y, radius;
float pitch1,roll1,throttle1;
float Yaw1 = 0.0;
// new these 3 parameters for pid control. pidx for roll, pidy for pitch, pidz for throttle.
PidController pidx,pidy,pidz;
bool pidflag = true;

bool takeoff_flag = false;
bool land_flag = false;

int center_count = 0;

///////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  //targetx=320,targetz=240,targety=r，如果程序循环执行，则setpoint每次只能执行一次
  if(pidflag){
    pidx.setPoint(160,0.05,0,0);
    pidy.setPoint(120,0.05,0,0);
    pidz.setPoint(120,0.05,0,0);
    pidflag = false;
  }
  ros::init(argc, argv, "demo_local_position_control_node");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  //ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
  ros::Subscriber gpsHealth      = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);

/////////////////////////////////////////////////////////////////////////////////////////////////////
  flag = (DJISDK::VERTICAL_VELOCITY   |
              DJISDK::HORIZONTAL_VELOCITY |
              DJISDK::YAW_RATE            |
              DJISDK::HORIZONTAL_BODY   |
              DJISDK::STABLE_ENABLE);

  ctrlFlightPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe( "/circle/position", 1, imageCalllback );

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);   //dji_sdk/flight_control_setpoint_ENUvelocity_yawrate
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  // if (!set_local_position())
  // {
  //   ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
  //   return 1;
  // }

  if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();  
    takeoff_flag = true;
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = monitoredTakeoff();
  }

    ros::spin();
    return 0;
  }



void imageCalllback(const sensor_msgs::ImageConstPtr& msg) 
{
    cv::Mat img;
    float center_distance=0;
    //ROS_INFO("Received \n");
    img = cv_bridge::toCvShare(msg, "mono8")->image;
    circle_x=(int)img.at<uchar>(0,0);
    circle_y=(int)img.at<uchar>(1,0);
    radius = (int)img.at<uchar>(2,0);
    //std::cout<<circle_x<<","<<circle_y<<","<<radius<<std::endl;
   

    //roll对应x，throttle对应y，pitch对应z
    // roll1 = pidx.control(circle_x);
    // throttle1 = pidy.control(circle_y);
    // pitch1 = 1.0*pidz.control(radius);
     roll1 = -0.01*(circle_x-160);
     throttle1 = -0.005*(circle_y-120);
     pitch1 = -0.02*(radius-120);


    //ROS_INFO("roll: %.2f, pitch: %.2f, thro: %.2f",roll1,pitch1,throttle1);
    ROS_INFO("takeoff_flag: %d",takeoff_flag);
    if(takeoff_flag)
    {
      ////////////////////////////////////////////////////////////
      controlVelYawRate.axes.push_back(0);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(roll1);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(throttle1);     //throttle: up
      controlVelYawRate.axes.push_back(0);     //yaw
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();                //Very Important!!!!
      ROS_INFO("control \n");
  ////////////////////////////////////////////////////////////

      ROS_INFO("circle_x: %d, circle_y: %d, radius: %d",circle_x,circle_y,radius);
      ROS_INFO("roll: %.2f, pitch: %.2f, thro: %.2f",roll1,pitch1,throttle1);

      center_distance = (float)(circle_x-160)*(circle_x-160)+(circle_y-120)*(circle_y-120);
      ROS_INFO("Distance_Center: %.2f",center_distance);

      if(center_distance<100) {
        center_count++; 
        ROS_INFO("CenterCount: %d",center_count);
      }
      if(center_count>10000) {
        land_flag = true;
        center_count=0;
      }

      if(land_flag)
        {
          ROS_INFO("M100 Landing!");
          M100monitoredLand();
        }
    }

   
    
} 


bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

//////////////////////////////Land//////////////////////////
bool landing_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("landing_land fail");
    return false;
  }
  return true;
}


bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}


void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps_position = *msg;
}

void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
  current_gps_health = msg->data;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
         (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
         ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps_position.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
     current_gps_position.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);

  return (bool)localPosReferenceSetter.response.result;
}




////////////////////////////Landing monitored///////////////////////////////////////
bool
M100monitoredLand()
{
  ros::Time start_time = ros::Time::now();

  // float home_altitude = current_gps.altitude;
  if(!landing_land(dji_sdk::DroneTaskControl::Request::TASK_LAND))
  {
    return false;
  }

  // ros::Duration(0.01).sleep();
  // ros::spinOnce();

  // // Step 1: If M100 is not in the air after 10 seconds, fail.
  // while (ros::Time::now() - start_time < ros::Duration(10))
  // {
  //   ros::Duration(0.01).sleep();
  //   ros::spinOnce();
  // }

  // if(flight_status == DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
  //     current_gps.altitude - home_altitude > 0.2)
  // {
  //   ROS_ERROR("Landing failed.");
  //   return false;
  // }
  // else
  // {
  //   start_time = ros::Time::now();
  //   ROS_INFO("Successful landing!");
  //   ros::spinOnce();
  // }

  return true;
}