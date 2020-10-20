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
#include "cmath"
#include "time.h"
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


///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
int circle_x, circle_y, radius;
int all_tag_count = 0;
int search_tag_count = 0;
int tag_fly_count = 0;
float pitch1,roll1,throttle1;
float pitch2 = 0;
float roll2,throttle2,yaw2;
float Yaw1 = 0.0;
float exp_altitude = 1.5;
float right_forward_angle = 0;
float right_back_angle = 0;
float left_forward_angle = 0;
float left_back_angle = 0;
float right_forward_distance = 0;
float left_forward_distance = 0;
float temp_right_forward_distance = 0;
float temp_left_forward_distance = 0;
float forward_left_distance = 0;
float forward_right_distance = 0;
float back_left_distance = 0;
float back_right_distance = 0;
float left_boundary_distane= 0;
float right_boundary_distane= 0;
float forward_boundary_distane= 0;
float back_boundary_distane= 0;
float now_time = 0;
float right_back_distance = 0;
float left_back_distance = 0;
float original_distance = 0;
float scanrange_radian = 0;
float forward_speed = 0.3;
float dji_altitude = 0;
float back_last_distance = 0;
float forward_last_distance = 0;
int direction_y = -1;
bool search_circle_flag = false;
bool finished_cross_flag = true;
bool first_circle_flag = false;
bool begin_cross_flag = false;
bool has_seem_flag = false;
bool search_tag_flag = false;
bool cross_a_tag_flag = false;
bool first_direction_flag = false;
int right_angle = 0;
int left_angle = 0;
int circle_count = 0;
int boundary_direction = 0;

// clock_t cross_begin_time;
ros::Time cross_time;
// new these 3 parameters for pid control. pidx for roll, pidy for pitch, pidz for throttle.
PidController pidx,pidy,pidz;

//////////////////////////flag space///////////////////////////////////////
bool pidflag = true;

bool takeoff_flag = false;
bool land_flag = false;
bool cross_flag = false;
bool scan_adjust_flag = false;
bool start_flag = false;
bool landing_flag = false;

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
  ros::Subscriber DisplayModeSub = nh.subscribe("dji_sdk/Display_mode", 10, &display_mode_callback);
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
  // image_transport::Subscriber sub = it.subscribe( "/circle/position", 1, imageCalllback );

  ros::Subscriber scanSub = nh.subscribe("/control/velocity", 1, &velocityCallback);

  ros::Subscriber heightSub = nh.subscribe("/dji_sdk/height_above_takeoff", 1, &heightCallback);

  // ros::Subscriber odomSub = nh.subscribe("/camera/odom/sample", 1, &odomCallback);


  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);   //dji_sdk/flight_control_setpoint_ENUvelocity_yawrate
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;


  if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();  
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = monitoredTakeoff();
  }

    ros::spin();
    return 0;
  }



void velocityCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{

 // right_forward_distancey = 0;
  //left_forward_distancey = 0;
  //change the data of A3 lidar!!!!!!!!!!!!! 
  //   original_distance-->meter   original_angle-->radian

  float velocity_x = msg->data[0];
  float velocity_y = msg->data[1];
  float velocity_z = msg->data[2];
  

  float pitch= velocity_x*0.3;
  float direction_y = velocity_y*0.3;
  float throttle = velocity_z*0.3;
  if (start_flag && dji_altitude<4){
      pitch = speedLimit(-0.3,pitch,0.3);
      direction_y = speedLimit(-0.3,direction_y,0.3);
      throttle = speedLimit(-0.3,throttle,0.3);

      std::cout<<pitch<<","<<direction_y<<","<<throttle<<std::endl;

      controlVelYawRate.axes.push_back(pitch);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(direction_y);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(throttle);     //throttle: up
      controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();
  } else {
      controlVelYawRate.axes.push_back(0);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(0);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(0);     //throttle: up
      controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();
      std::cout<<"hold on !!!!"<<std::endl;

  }



          
      // case 10:{
      //   ROS_INFO("case1010101010101011010101010\n");
      //  while (ros::Time::now() - cross_time < ros::Duration(5))
      // {
      //   pitch2 = 0;
      //   if(forward_last_distance<1.3 && forward_last_distance != 0){
      //     pitch2 = -0.1;
      //   }
      //   if(back_last_distance<1.3 && back_last_distance != 0){
      //     pitch2 = 0.1;
      //   }
      //   throttle2 = 1.1-dji_altitude;         
      //   throttle2 = speedLimit(-0.3,throttle2,0.3);
      //   controlVelYawRate.axes.push_back(0);     //pitch: forward --> positive
      //   controlVelYawRate.axes.push_back(-0.3);     //roll:  left    --> positive
      //   controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      //   controlVelYawRate.axes.push_back(0);     //yaw
      //   controlVelYawRate.axes.push_back(flag);
      //   ctrlFlightPub.publish(controlVelYawRate);
      //   ros::Duration(0.01).sleep();

      //   ROS_INFO("cross control_gogogo \n");
      //   // ros::spinOnce();
      // }
      // if(ros::Time::now() - cross_time > ros::Duration(5)){ 
      //   controlVelYawRate.axes.clear();                //Very Important!!!!
      //   M100monitoredLand();
      //   ROS_INFO("landing!!!!!-------------___!!!!!!!!!!!!!!!!!!!----");
      // }
      // }
 
////////////////////////////////switch end

  //////////////////////////////////search tag end
}

void heightCallback(const std_msgs::Float32::ConstPtr& msg)
{
  if(takeoff_flag){
    dji_altitude = msg->data;
    // ROS_INFO("dji_altitude: %.2f",dji_altitude);
  }
}

// void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
// {
//   distance_t265_x = msg->pose.pose.position.x;
//   distance_t265_y = msg->pose.pose.position.y;
//   // ROS_INFO("position: %.2f",distance_t265_y);
// }

float speedLimit(float min_Speed,float speed,float max_Speed) {
  if(speed>max_Speed) speed = max_Speed;
  if(speed<min_Speed) speed = min_Speed;
  return speed;
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
 * and the more detailed Display_mode (only for A3/N3)
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
// || current_gps_position.altitude - home_altitude < 1.0
  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    takeoff_flag = true;
    start_time = ros::Time::now();
    ros::spinOnce();
  }
  if(takeoff_flag && !start_flag) {

    while (ros::Time::now() - start_time < ros::Duration(4))
    {
      ROS_INFO("dji_altitude: %.2f",dji_altitude);
      throttle2 = 1.0-dji_altitude;         
      throttle2 = speedLimit(-0.3,throttle2,0.3);
      controlVelYawRate.axes.push_back(0);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(0);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      controlVelYawRate.axes.push_back(0);     //yaw
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }
    start_flag = true;
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

