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

  ros::Subscriber scanSub = nh.subscribe("/circle/scan", 1, &laserCallback);

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


// void imageCalllback(const sensor_msgs::ImageConstPtr& msg) 
// {
    // cv::Mat img;
    // float center_distance=0;
    // //ROS_INFO("Received \n");
    // img = cv_bridge::toCvShare(msg, "mono8")->image;
    // circle_x=(int)img.at<uchar>(0,0);
    // circle_y=(int)img.at<uchar>(1,0);
    // radius = (int)img.at<uchar>(2,0);
    // //std::cout<<circle_x<<","<<circle_y<<","<<radius<<std::endl;
   

    // //roll对应x，throttle对应y，pitch对应z
    // // roll1 = pidx.control(circle_x);
    // // throttle1 = pidy.control(circle_y);
    // // pitch1 = 1.0*pidz.control(radius);
    // if(radius<255) {
    //     roll1 = -0.01*(circle_x-160);
    //     throttle1 = -0.005*(circle_y-120);
    //     pitch1 = -0.02*(radius-160);                //radius = 160 , distance = 2.3m
    // } else {
    //     roll1 = 0;
    //     throttle1 = 0;
    //     pitch1 = 0;                //radius = 160 , distance = 2.3m
    // }


    //ROS_INFO("roll: %.2f, pitch: %.2f, thro: %.2f",roll1,pitch1,throttle1);
  //   ROS_INFO("takeoff_flag: %d",takeoff_flag);
  //   if(takeoff_flag && !scan_adjust_flag)
  //   {
  //     ////////////////////////////////////////////////////////////
  //     controlVelYawRate.axes.push_back(pitch1);     //pitch: forward --> positive
  //     controlVelYawRate.axes.push_back(roll1);     //roll:  left    --> positive
  //     controlVelYawRate.axes.push_back(throttle1);     //throttle: up
  //     controlVelYawRate.axes.push_back(0);     //yaw
  //     controlVelYawRate.axes.push_back(flag);
  //     ctrlFlightPub.publish(controlVelYawRate);
  //     controlVelYawRate.axes.clear();                //Very Important!!!!
  //     ROS_INFO("Image Control \n");
  // ////////////////////////////////////////////////////////////

  //     ROS_INFO("circle_x: %d, circle_y: %d, radius: %d",circle_x,circle_y,radius);
  //     ROS_INFO("roll1: %.2f, pitch1: %.2f, thro1: %.2f",roll1,pitch1,throttle1);

  //     center_distance = (float)(circle_x-160)*(circle_x-160)+(circle_y-120)*(circle_y-120);
  //     ROS_INFO("distance_Center: %.2f",center_distance);

  //     if(center_distance<100 && abs(radius-160)<30) {
  //       center_count++; 
  //       ROS_INFO("CenterCount: %d",center_count);
  //     }
  //     if(center_count>150) {
  //       // land_flag = true;
  //       scan_adjust_flag = true;
  //       center_count=0;
  //     }

  //   } 
    // if(scan_adjust_flag&&takeoff_flag&&search_circle_flag&&!finished_cross_flag) {
    //   if(fabs(rightdistance)<0.55||fabs(leftdistance)<0.55) {
    //     forward_speed = 0;
    //     ROS_INFO("ERROR");
    //   } else {
    //       forward_speed = 0.3;
    //   }
    //   roll2 = right_forward_distance+left_forward_distance;
    //   throttle2 = 1.3-dji_altitude;
    //   controlVelYawRate.axes.push_back(forward_speed);     //pitch: forward --> positive
    //   controlVelYawRate.axes.push_back(roll2);     //roll:  left    --> positive
    //   controlVelYawRate.axes.push_back(throttle2);     //throttle: up
    //   controlVelYawRate.axes.push_back(0);     //yaw
    //   controlVelYawRate.axes.push_back(flag);
    //   ctrlFlightPub.publish(controlVelYawRate);
    //   controlVelYawRate.axes.clear();                //Very Important!!!!
    //   ROS_INFO("leftdistance: %.2f, rightdistance: %.2f",leftdistance,rightdistance);
    //   ROS_INFO("scan adjust control \n");

    //   ROS_INFO("roll2: %.2f, pitch2: %.2f, thro2: %.2f",roll2,forward_speed,throttle2);

    //   if(left_angle>50 && abs(rightdistance)>0.6 && abs(leftdistance)>0.6) {
    //     cross_flag = true;
    //     scan_adjust_flag = false;
    //     first_circle_flag = true;
    //     ROS_INFO("Ready to cross!");
    //   }
    // }

    // if(takeoff_flag && cross_flag) {
    //   controlVelYawRate.axes.push_back(0.3);     //pitch: forward --> positive
    //   controlVelYawRate.axes.push_back(0);     //roll:  left    --> positive
    //   controlVelYawRate.axes.push_back(0);     //throttle: up
    //   controlVelYawRate.axes.push_back(0);     //yaw
    //   controlVelYawRate.axes.push_back(flag);
    //   ctrlFlightPub.publish(controlVelYawRate);
    //   controlVelYawRate.axes.clear();                //Very Important!!!!
    //   ROS_INFO("leftdistance: %.2f, rightdistance: %.2f",leftdistance,rightdistance);
    //   ROS_INFO("cross control \n");
    //   // if(left_angle>140) {
    //   //   land_flag = true;
    //   //   cross_flag = false;
    //   // }
    // }

    // if(circle_count ==3)
    //   {
    //     ROS_INFO("M100 Landing!");
    //     M100monitoredLand();
    //   }   
// } 

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  right_forward_distance = 0;
  right_forward_angle = 0;
  right_back_angle = 0;
  right_back_distance = 0;
  forward_right_distance = 0;
  back_right_distance = 0;
  temp_right_forward_distance = 0;
  temp_left_forward_distance = 0;

  left_forward_angle = 0;
  left_forward_distance = 0;
  left_back_angle = 0;
  left_back_distance = 0;
  forward_left_distance = 0;  
  back_left_distance = 0;
  left_boundary_distane = 0;
  back_last_distance = 0;
  forward_last_distance = 0;
  right_boundary_distane = 0;
 // right_forward_distancey = 0;
  //left_forward_distancey = 0;
  //change the data of A3 lidar!!!!!!!!!!!!! 
  //   original_distance-->meter   original_angle-->radian
  std::vector<float> scandistance=msg->ranges;
  std::vector<float> scanrange=msg->intensities;
  if(!search_tag_flag){
  for(int i=0;i<scandistance.size();i++)
  {
    scanrange_radian = scanrange[i]/180*3.14159;
    if(scanrange[i]>290&&scanrange[i]<350){
      temp_right_forward_distance = scandistance[i]*sin(scanrange_radian);
      if(scandistance[i]*sin(scanrange_radian)>-1 && scandistance[i]*sin(scanrange_radian)<-0.4){
        right_forward_angle = scanrange[i];
        right_forward_distance = scandistance[i]*sin(scanrange_radian);
        forward_right_distance = fabs(scandistance[i]*cos(scanrange_radian));

      }
      //right_forward_distancey = fabs(right_forward_distance/)
    }
    if(scanrange[i]>170&&scanrange[i]<250){
      if(scandistance[i]*sin(scanrange_radian)>-0.85 && scandistance[i]*sin(scanrange_radian)<-0.55){
        right_back_angle = scanrange[i];
        right_back_distance = fabs(scandistance[i]*sin(scanrange_radian));
      }
    }
    if(scanrange[i]>15 && scanrange[i]<82) {
       temp_left_forward_distance = scandistance[i]*sin(scanrange_radian);
      if(scandistance[i]*sin(scanrange_radian)<1.0 && scandistance[i]*sin(scanrange_radian)>0.4){
        left_forward_angle = scanrange[i];
        left_forward_distance = scandistance[i]*sin(scanrange_radian);
        forward_left_distance = fabs(scandistance[i]*cos(scanrange_radian));
        }
    }
    if(scanrange[i]<165&&scanrange[i]>105) {
      if(scandistance[i]*sin(scanrange_radian)<0.85 && scandistance[i]*sin(scanrange_radian)>0.55){
        left_back_angle = scanrange[i];
        left_back_distance = fabs(scandistance[i]*sin(scanrange_radian));
      }
    }
    if(scanrange[i]>75 && scanrange[i]<105 && scandistance[i]>0.4){
      if(left_boundary_distane == 0 || (scandistance[i] < left_boundary_distane) ){
          left_boundary_distane = scandistance[i];
        }
    }
    if(scanrange[i]>255 && scanrange[i]<285 && scandistance[i]>0.4){
      if(right_boundary_distane == 0 || (scandistance[i] < right_boundary_distane) ){
        right_boundary_distane = scandistance[i];
      }
    }
    if(scanrange[i]>174 && scanrange[i]<186 && scandistance[i]>0.4){
      if(back_last_distance == 0 || (scandistance[i] < back_last_distance) ){
        back_last_distance = scandistance[i];
      }
    }
    if((scanrange[i]>0 && scanrange[i]<10) || (scanrange[i]>350 && scanrange[i]<360 && scandistance[i]>0.4)){
      if(forward_last_distance == 0 || (scandistance[i] < forward_last_distance) ){
        forward_last_distance = scandistance[i];
      }
    }
  }
  //2 meter first
  
//判断是否看到环
  if(((start_flag && right_forward_distance != 0 && left_forward_distance != 0) 
      || has_seem_flag)  && finished_cross_flag ){
    ROS_INFO("Find one circle");
    // when find circle
    //search_circle_flag = true;
    has_seem_flag = true;
    scan_adjust_flag = true;
    finished_cross_flag = false;

    //begin_cross_flag = true;
    //choose a suitable altitude
    // if(fabs(right_forward_distance)+fabs(left_forward_distance)<1.1){
    //   if(dji_altitude<1.5){
    //     exp_altitude=1.3;
    //   }
    //   if(dji_altitude>=1.5){
    //     exp_altitude=1.3;
    //   }
    // }
  // }else if((left_back_angle>135 || right_back_angle<225) && first_circle_flag && cross_flag){
  //   //has been crossed the circle
  //   search_circle_flag = false;
  //   cross_flag = false;
  //   scan_adjust_flag = false;
  //   finished_cross_flag = true;
  //   has_seem_flag = false;
  //   ROS_INFO("Ready to cross!");

  }

///////////////////////////////////////// for search circle

  if(start_flag && !has_seem_flag){
      ROS_INFO("Find no circle");
    //has_seem_flag: true --> none cercle detect
      // if((temp_right_forward_distance + temp_left_forward_distance) > 0 
      //     && temp_left_forward_distance>0 && temp_right_forward_distance >0){
      //   direction_y = 1;
      // }
      // if((temp_right_forward_distance + temp_left_forward_distance) < 0 
      //     && temp_left_forward_distance>0 && temp_right_forward_distance >0){
      //   direction_y = -1;
      // }
      pitch2 = 0;
      if((circle_count == 0 ||circle_count == 3 || circle_count == 5) && !first_direction_flag){
        direction_y = -1; 
	first_direction_flag = true;
        }
      if((circle_count == 1 || circle_count == 2 || circle_count == 4) && !first_direction_flag ){
        direction_y = 1; 
	first_direction_flag = true;
        }
      ROS_INFO("circlr_count: %d",circle_count);
      ROS_INFO("left_boundary_distane: %.2f",left_boundary_distane);
      ROS_INFO("right_boundary_distane: %.2f",right_boundary_distane);
      if( left_boundary_distane<1.3 && left_boundary_distane !=0){
        direction_y = -1;
      }
      if( right_boundary_distane<1.3 && right_boundary_distane !=0){
        direction_y = 1;
      }
      if(forward_last_distance<1 && forward_last_distance !=0){
          pitch2 = -0.1;
      }
      if(back_last_distance<1 && back_last_distance !=0){
          pitch2 = 0.1;
      }
      pitch2 = speedLimit(-0.12,pitch2,0.12);
      ROS_INFO("direction_y: %d",direction_y);
      throttle2 = 1.3-dji_altitude;
      throttle2 = speedLimit(-0.3,throttle2,0.3);
      controlVelYawRate.axes.push_back(pitch2);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(0.3*direction_y);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();
      
      //5 is the boundary for search,it will be replaced in the real scence
      //t265 left-->positive
      
    }
    
///////////////////////////////////////// for search circle

    if(scan_adjust_flag && start_flag && has_seem_flag) {
      if((fabs(right_forward_distance)<0.55||fabs(left_forward_distance)<0.55)
      &&right_forward_distance != 0&&left_forward_distance != 0) {
        forward_speed = 0;
        ROS_INFO("ERROR,do not go ahead ,just adjust the distance of the left and the right");
      } else {
          forward_speed = 0.3;
      }
      roll2 = right_forward_distance+left_forward_distance;
      roll2 = speedLimit(-0.3,roll2,0.3);
      throttle2 =1.3-dji_altitude;
      yaw2 = forward_right_distance - forward_left_distance;
      throttle2 = speedLimit(-0.3,throttle2,0.3);
      yaw2 = speedLimit(-0.1,yaw2,0.1);
      controlVelYawRate.axes.push_back(forward_speed);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(roll2);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      controlVelYawRate.axes.push_back(0.3*yaw2);     //yaw
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();                //Very Important!!!!
      //ROS_INFO("left_forward_distance: %.2f, right_forward_distance: %.2f,%.1f,%.1f",left_forward_distance,right_forward_distance,left_forward_angle,right_forward_angle);
      ROS_INFO("scan adjust control \n");

      // ROS_INFO("roll2: %.2f, pitch2: %.2f, thro2: %.2f",roll2,forward_speed,throttle2);

      if((left_forward_angle>55 && right_forward_angle<305) && (fabs(right_forward_distance)>0.6 
      && fabs(left_forward_distance)>0.6) && !cross_flag){
        cross_flag = true;
        scan_adjust_flag = false;
        first_circle_flag = true;
        has_seem_flag = false; 
        ROS_INFO("Ready to cross!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n!!!\n!!!!!!!!!!!!!!!!");
        // cross_begin_time = clock();
        cross_time = ros::Time::now();
      }
    }

    if(start_flag && cross_flag) {
      while (ros::Time::now() - cross_time < ros::Duration(4))
      {
        if(dji_altitude<1.0){
          dji_altitude = 1.3;
        }
        throttle2 = 1.3-dji_altitude;         
        throttle2 = speedLimit(-0.3,throttle2,0.3);
        controlVelYawRate.axes.push_back(0.33);     //pitch: forward --> positive
        controlVelYawRate.axes.push_back(0);     //roll:  left    --> positive
        controlVelYawRate.axes.push_back(throttle2);     //throttle: up
        controlVelYawRate.axes.push_back(0);     //yaw
        controlVelYawRate.axes.push_back(flag);
        ctrlFlightPub.publish(controlVelYawRate);
        ros::Duration(0.01).sleep();

        ROS_INFO("cross control_gogogo \n");
        // ros::spinOnce();
      }
      if(ros::Time::now() - cross_time > ros::Duration(4)) {
        search_circle_flag = false;
        cross_flag = false;
        has_seem_flag = false;
        circle_count++;
        finished_cross_flag = true;
        first_direction_flag = false;
        controlVelYawRate.axes.clear();                //Very Important!!!!

        ROS_INFO("Circle_count: %d     -------------___!!!!!!!!!!!!!!!!!!!----",circle_count);
      }
      // ROS_INFO("Circle_count: %d, diff_time: %.2f",circle_count,difftime(clock(),cross_begin_time));
      // if(dji_altitude<1.0){
      //   dji_altitude = 1.3;
      // }
    //     throttle2 = 1.3-dji_altitude;
              
    //   throttle2 = speedLimit(-0.3,throttle2,0.3);
    //   controlVelYawRate.axes.push_back(0.3);     //pitch: forward --> positive
    //   controlVelYawRate.axes.push_back(0);     //roll:  left    --> positive
    //   controlVelYawRate.axes.push_back(0);     //throttle: up
    //   controlVelYawRate.axes.push_back(0);     //yaw
    //   controlVelYawRate.axes.push_back(flag);
    //   ctrlFlightPub.publish(controlVelYawRate);
    //   // controlVelYawRate.axes.clear();                //Very Important!!!!
    //  // ROS_INFO("left_forward_distance: %.2f, right_forward_distance: %.2f,%.1f,%.1f",left_forward_distance,right_forward_distance,left_forward_angle,right_forward_angle);
      // if(left_angle>140) {
      //   land_flag = true;
      //   cross_flag = false;
      // }
    }

    if(circle_count == 6) {
      search_tag_flag = true;
      // ROS_INFO("M100 Landing!");
      // M100monitoredLand();
    }   
   // ROS_INFO("range: %d, distance: %.2f",(int)scanrange[i],scandistance[i]);
   //ROS_INFO("leftdistance: %.2f, rightdistance: %.2f",leftdistance,rightdistance);
  }
  ///////////////////////search tag begin
  if(search_tag_flag){
    for(int i=0;i<scandistance.size();i++)
    {
      scanrange_radian = scanrange[i]/180*3.14159;
      if(scandistance[i]<2.7)
      {
        if(scanrange[i]>290&&scanrange[i]<355){
          temp_right_forward_distance = scandistance[i]*sin(scanrange_radian);
         
            right_forward_angle = scanrange[i];
            right_forward_distance = scandistance[i]*sin(scanrange_radian);
            forward_right_distance = fabs(scandistance[i]*cos(scanrange_radian));

          
          //right_forward_distancey = fabs(right_forward_distance/)
        }
        if(scanrange[i]>185&&scanrange[i]<250){
          right_back_angle = scanrange[i];
          right_back_distance = fabs(scandistance[i]*sin(scanrange_radian));
          back_right_distance = fabs(scandistance[i]*cos(scanrange_radian));
        }
        if(scanrange[i]>5 && scanrange[i]<70) {
          temp_left_forward_distance = scandistance[i]*sin(scanrange_radian);
          left_forward_angle = scanrange[i];
          left_forward_distance = scandistance[i]*sin(scanrange_radian);
          forward_left_distance = fabs(scandistance[i]*cos(scanrange_radian));
            
        }
        if(scanrange[i]<175&&scanrange[i]>110) {
          left_back_angle = scanrange[i];
          left_back_distance = fabs(scandistance[i]*sin(scanrange_radian));
          back_left_distance = fabs(scandistance[i]*cos(scanrange_radian));
        }
        if(scanrange[i]>75 && scanrange[i]<105 && scandistance[i]>0.4){
          if(left_boundary_distane == 0 || (scandistance[i] < left_boundary_distane) ){
              left_boundary_distane = scandistance[i];
            }
        }
        if(scanrange[i]>255 && scanrange[i]<285 && scandistance[i]>0.4){
          if(right_boundary_distane == 0 || (scandistance[i] < right_boundary_distane) ){
            right_boundary_distane = scandistance[i];
          }
        }
        if(scanrange[i]>174 && scanrange[i]<186 && scandistance[i]>0.4){
          if(back_last_distance == 0 || (scandistance[i] < back_last_distance) ){
            back_last_distance = scandistance[i];
          }
        }
        if((scanrange[i]>0 && scanrange[i]<10) || (scanrange[i]>350 && scanrange[i]<360 && scandistance[i]>0.4)){
          if(forward_last_distance == 0 || (scandistance[i] < forward_last_distance) ){
            forward_last_distance = scandistance[i];
          }
        }
      }
    }
    ///////////////////////////////scan filering over,search tag begin
    switch(tag_fly_count){
      case 0:{
        if(right_boundary_distane<1.4 && right_boundary_distane !=0){
          tag_fly_count = 1;
        }
        pitch2 = 0;
        if(forward_last_distance<0.9 && forward_last_distance !=0){
          pitch2 = -0.1;
        }
        if(back_last_distance<1  && back_last_distance !=0){
          pitch2 = 0.2;
        }
      pitch2 = speedLimit(-0.1,pitch2,0.2);
      throttle2 = 1.1-dji_altitude;
      throttle2 = speedLimit(-0.2,throttle2,0.2);
      controlVelYawRate.axes.push_back(pitch2);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(-0.4);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();
      break;
      }
      case 1:{
        ROS_INFO("case11111111111111111111111111 \n");
        if(left_boundary_distane<1.4 && left_boundary_distane != 0){
          tag_fly_count = 2;
          cross_time = ros::Time::now();
          break;
        }
        pitch2 = 0;
        if(forward_last_distance<1 && forward_last_distance != 0){
          pitch2 = -0.12;
        }
        if(back_last_distance<1 && back_last_distance != 0){
          pitch2 = 0.2;
        }
      pitch2 = speedLimit(-0.12,pitch2,0.2);
      throttle2 = 1.1-dji_altitude;
      throttle2 = speedLimit(-0.2,throttle2,0.2);
      controlVelYawRate.axes.push_back(pitch2);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(0.4);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();
      break;
      }
      case 2:{
        ROS_INFO("case2222222222222222222 \n");
        // if(right_boundary_distane != 0 && left_boundary_distane != 0){
        //   if(right_boundary_distane>left_boundary_distane){
        //     boundary_direction = -1;
        //   }else{
        //     boundary_direction = 1;
        //   }
        // }else{
        //   boundary_direction = 0;
        //   }
      //roll2 = fabs(right_boundary_distane-left_boundary_distane)*boundary_direction;
     // roll2 = speedLimit(-0.2,roll2,0.2);

    while (ros::Time::now() - cross_time < ros::Duration(1))
      {
          
    
        throttle2 = 1.1-dji_altitude;         
        throttle2 = speedLimit(-0.3,throttle2,0.3);
        controlVelYawRate.axes.push_back(0);     //pitch: forward --> positive
        controlVelYawRate.axes.push_back(0);     //roll:  left    --> positive
        controlVelYawRate.axes.push_back(throttle2);     //throttle: up
        controlVelYawRate.axes.push_back(0);     //yaw
        controlVelYawRate.axes.push_back(flag);
        ctrlFlightPub.publish(controlVelYawRate);
        ros::Duration(0.01).sleep();

        ROS_INFO("cross control_gogogo \n");
        // ros::spinOnce();
      }
      if(ros::Time::now() - cross_time > ros::Duration(1)){ 
        roll2 = -0.05;
        pitch2 = 0.3;
        if(left_boundary_distane<0.8 && left_boundary_distane != 0){
          roll2 = -0.2;
           pitch2 = 0.1;
        }
        if((right_boundary_distane<0.8 && right_boundary_distane != 0) ||
        (fabs(right_forward_distance)<0.7 && fabs(right_forward_distance) > 0.5)){
          roll2 = 0.2;
          pitch2 = 0.1;
        }
        if(right_boundary_distane>0.5 && !cross_a_tag_flag){
          cross_a_tag_flag = true;
          cross_time = ros::Time::now();
        }
        if(cross_a_tag_flag && ( (forward_last_distance<1.6 && forward_last_distance != 0) || 
        back_right_distance>1.0 || ros::Time::now() - cross_time > ros::Duration(7)) ){
          tag_fly_count = 3;
          cross_a_tag_flag = false;
          break;
        }
      ROS_INFO("Cright_forward_distance: %.2f________________",right_forward_distance);
      throttle2 = 1.1-dji_altitude;
      throttle2 = speedLimit(-0.2,throttle2,0.2);
      
      controlVelYawRate.axes.push_back(pitch2);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(roll2);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();
      break;
      }

      }
      case 3:{
        ROS_INFO("case33333333333333333333333 \n");
        if(right_boundary_distane<1.4 && right_boundary_distane !=0){
          tag_fly_count = 4;
          cross_time = ros::Time::now();
          break;
        }
        pitch2 = 0;
        if(forward_last_distance<1 && forward_last_distance > 0.5){
          pitch2 = -0.2;
        }
        if(back_last_distance<1 && back_last_distance >0.5 ){
          pitch2 = 0.2;
        }
      pitch2 = speedLimit(-0.2,pitch2,0.2);
      throttle2 = 1.1-dji_altitude;
      throttle2 = speedLimit(-0.2,throttle2,0.2);
      controlVelYawRate.axes.push_back(pitch2);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(-0.3);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();
      break;
      }
      case 4:{

        ROS_INFO("case444444444444444444 \n");
        // if(right_boundary_distane != 0 && left_boundary_distane != 0){
        //   if(right_boundary_distane>left_boundary_distane){
        //     boundary_direction = -1;
        //   }else{
        //     boundary_direction = 1;
        //   }
        // }else{
        //   boundary_direction = 0;
        //   }
      //roll2 = fabs(right_boundary_distane-left_boundary_distane)*boundary_direction;
     // roll2 = speedLimit(-0.2,roll2,0.2);

    while (ros::Time::now() - cross_time < ros::Duration(4))
      {
          
        pitch2 = 0;
        if(forward_last_distance<0.9 && forward_last_distance > 0.4){
          pitch2 = -0.1;
        }
        if(back_last_distance<1  && back_last_distance >0.4){
          pitch2 = 0.2;
        }
        throttle2 = 1.1-dji_altitude;         
        throttle2 = speedLimit(-0.3,throttle2,0.3);
        controlVelYawRate.axes.push_back(0);     //pitch: forward --> positive
        controlVelYawRate.axes.push_back(0.3);     //roll:  left    --> positive
        controlVelYawRate.axes.push_back(throttle2);     //throttle: up
        controlVelYawRate.axes.push_back(0);     //yaw
        controlVelYawRate.axes.push_back(flag);
        ctrlFlightPub.publish(controlVelYawRate);
        ros::Duration(0.01).sleep();

        // ROS_INFO("cross control_gogogo \n");
        // ros::spinOnce();
      }
      if(ros::Time::now() - cross_time > ros::Duration(4)){ 
        tag_fly_count = 5;
      }

      //   ROS_INFO("case4444444444444444444444444444 \n");
      //   //  if(right_boundary_distane != 0 && left_boundary_distane != 0){
      //   //   if(right_boundary_distane>left_boundary_distane){
      //   //     boundary_direction = -1;
      //   //   }else{
      //   //     boundary_direction = 1;
      //   //   }
      //   // }else{
      //   //   boundary_direction = 0;
      //   //   }
      //   //roll2 = fabs(right_boundary_distane-left_boundary_distane)*boundary_direction;
      //   //roll2 = speedLimit(-0.2,roll2,0.2);
        
      //   pitch2 = 0;
      //   if(forward_last_distance<1 && forward_last_distance != 0){
      //     pitch2 = -0.1;
      //   }
      //   if(back_last_distance<1 && back_last_distance != 0){
      //     pitch2 = 0.1;
      //   }
      //   if(forward_last_distance>0.65){
      //     cross_a_tag_flag = true;
      //   }
      //   if(cross_a_tag_flag && ((fabs(left_forward_distance) <1.3 && fabs(left_forward_distance) >0)||
      //   (fabs(left_back_distance) <0.7 && fabs(left_back_distance) >0) ) ){
      //     tag_fly_count = 5;
      //     cross_a_tag_flag = false;
      //     break;
      //   }
      // throttle2 = 1.1-dji_altitude;
      // throttle2 = speedLimit(-0.2,throttle2,0.2);
      // controlVelYawRate.axes.push_back(pitch2);     //pitch: forward --> positive
      // controlVelYawRate.axes.push_back(0.3);     //roll:  left    --> positive
      // controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      // controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      // controlVelYawRate.axes.push_back(flag);
      // ctrlFlightPub.publish(controlVelYawRate);
      // controlVelYawRate.axes.clear();
      // break;
      }
      case 5:{
        ROS_INFO("case555555555555555555555555 \n");
        //  if(left_boundary_distane<0.8 && left_boundary_distane != 0){
        //   tag_fly_count = 6;
        // }
        // pitch2 = 0;
        roll2 = 0;
        if(left_boundary_distane<1 && left_boundary_distane != 0){
          roll2 = -0.15;
        }
        if(right_boundary_distane<1 && right_boundary_distane != 0){
          roll2 = 0.15;
        }
        if(left_boundary_distane > 0.8 && left_boundary_distane <2){
          cross_a_tag_flag = true;
        }
        if(cross_a_tag_flag && (back_left_distance>1.8 || back_right_distance >1.8) ) {
          cross_a_tag_flag = false;
          tag_fly_count = 7;
          break;
        }
      pitch2 = speedLimit(-0.12,pitch2,0.12);
      throttle2 = 1.1-dji_altitude;
      throttle2 = speedLimit(-0.2,throttle2,0.2);
      controlVelYawRate.axes.push_back(0.2);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(roll2);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();
      break;
      }
      // case 6:{
      //   ROS_INFO("case666666666666666666666666666\n");
      //   if(right_boundary_distane<1.2 && right_boundary_distane != 0){
      //     tag_fly_count = 7;
      //     break;
      //   }
      //   pitch2 = 0;
      //   if(forward_last_distance<1 && forward_last_distance != 0){
      //     pitch2 = -0.1;
      //   }
      //   if(back_last_distance<1  && back_last_distance != 0){
      //     pitch2 = 0.1;
      //   }
      // pitch2 = speedLimit(-0.12,pitch2,0.12);
      // throttle2 = 1.0-dji_altitude;
      // throttle2 = speedLimit(-0.2,throttle2,0.2);
      // controlVelYawRate.axes.push_back(pitch2);     //pitch: forward --> positive
      // controlVelYawRate.axes.push_back(-0.3);     //roll:  left    --> positive
      // controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      // controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      // controlVelYawRate.axes.push_back(flag);
      // ctrlFlightPub.publish(controlVelYawRate);
      // controlVelYawRate.axes.clear();
      // break;
      //   //  if(right_boundary_distane != 0 && left_boundary_distane != 0){
      //   //   if(right_boundary_distane>left_boundary_distane){
      //   //     boundary_direction = -1;
      //   //   }else{
      //   //     boundary_direction = 1;
      //   //   }
      //   // }else{
      //   //   boundary_direction = 0;
      //   //   }
      //   // roll2 = fabs(right_boundary_distane-left_boundary_distane)*boundary_direction;
      // //   roll2 = 0;
      // //   if(left_boundary_distane<0.8 && left_boundary_distane != 0){
      // //     roll2 = -0.1;
      // //   }
      // //   if(right_boundary_distane<0.8 && right_boundary_distane != 0){
      // //     roll2 = 0.1;
      // //   }
        
      // //   if(cross_a_tag_flag && forward_right_distance <1.5 && forward_right_distance != 0){
      // //     tag_fly_count = 7;
      // //     cross_a_tag_flag = false;
      // //     break;
      // //   }
      // // throttle2 = 1.0-dji_altitude;
      // // throttle2 = speedLimit(-0.2,throttle2,0.2);
      // // roll2 = speedLimit(-0.2,roll2,0.2);
      // // controlVelYawRate.axes.push_back(0.2);     //pitch: forward --> positive
      // // controlVelYawRate.axes.push_back(roll2);     //roll:  left    --> positive
      // // controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      // // controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      // // controlVelYawRate.axes.push_back(flag);
      // // ctrlFlightPub.publish(controlVelYawRate);
      // // controlVelYawRate.axes.clear();
      // // break;
      // }
      case 7:{
        ROS_INFO("case7777777777777777777777\n");
        if(left_boundary_distane<1.4 && left_boundary_distane != 0){
          tag_fly_count = 8;
          cross_time = ros::Time::now();
          break;
        }
        pitch2 = 0;
        if(forward_last_distance<1 && forward_last_distance != 0){
          pitch2 = -0.2;
        }
        if(back_last_distance<1 && back_last_distance != 0){
          pitch2 = 0.2;
        }
      pitch2 = speedLimit(-0.12,pitch2,0.12);
      throttle2 = 1.1-dji_altitude;
      throttle2 = speedLimit(-0.2,throttle2,0.2);
      controlVelYawRate.axes.push_back(pitch2);     //pitch: forward --> positive
      controlVelYawRate.axes.push_back(0.3);     //roll:  left    --> positive
      controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      controlVelYawRate.axes.push_back(flag);
      ctrlFlightPub.publish(controlVelYawRate);
      controlVelYawRate.axes.clear();
      break;
      //    if(back_last_distance>1){
      //      cross_a_tag_flag = true;
      //     // ROS_INFO("M100 Landing!");
      //     // M100monitoredLand();
      //   }
      //   if(left_back_distance > 0.7 && left_back_distance < 1.4 && right_back_distance > 0.7 && right_back_distance < 1.4  
      //   && cross_a_tag_flag){
      //     landing_flag = true;
      //     cross_a_tag_flag = false;
      //   }
      //   if(landing_flag && cross_a_tag_flag){
      //     ROS_INFO("M100 Landing!");
      //     M100monitoredLand();
      //   }
      // pitch2 = forward_last_distance - 1.4;
      // pitch2 = speedLimit(-0.2,pitch2,0.2);
      // throttle2 = 1.0-dji_altitude;
      // throttle2 = speedLimit(-0.2,throttle2,0.2);
      // controlVelYawRate.axes.push_back(pitch2);     //pitch: forward --> positive
      // controlVelYawRate.axes.push_back(-0.3);     //roll:  left    --> positive
      // controlVelYawRate.axes.push_back(throttle2);     //throttle: up
      // controlVelYawRate.axes.push_back(0);     //yaw positive->nishizhen max 0.1
      // controlVelYawRate.axes.push_back(flag);
      // ctrlFlightPub.publish(controlVelYawRate);
      // controlVelYawRate.axes.clear();
      // break;
      }
      case 8:{
        ROS_INFO("case888888888888888\n");
         
        while (ros::Time::now() - cross_time < ros::Duration(10))
      {
          
        pitch2 = 0;
        if(forward_last_distance<0.9 && forward_last_distance > 0.4){
          pitch2 = -0.2;
        }
        if(back_last_distance<1  && back_last_distance >0.4){
          pitch2 = 0.2;
        }
        throttle2 = 1.1-dji_altitude;         
        throttle2 = speedLimit(-0.3,throttle2,0.3);
        controlVelYawRate.axes.push_back(pitch2);     //pitch: forward --> positive
        controlVelYawRate.axes.push_back(-0.3);     //roll:  left    --> positive
        controlVelYawRate.axes.push_back(throttle2);     //throttle: up
        controlVelYawRate.axes.push_back(0);     //yaw
        controlVelYawRate.axes.push_back(flag);
        ctrlFlightPub.publish(controlVelYawRate);
        ros::Duration(0.01).sleep();

        // ROS_INFO("cross control_gogogo \n");
        // ros::spinOnce();
      }
      if(ros::Time::now() - cross_time > ros::Duration(4)){ 
       M100monitoredLand();
      }

      }
      case 9:{
        ROS_INFO("case999999999999999\n");
        //  if(left_boundary_distane<0.8 && left_boundary_distane != 0){
        //   tag_fly_count = 6;
        // }
        // pitch2 = 0;
        while (ros::Time::now() - cross_time < ros::Duration(4))
      {
          
        roll2 = 0;
        if(left_boundary_distane<1 && left_boundary_distane != 0){
          roll2 = -0.15;
        }
        if(right_boundary_distane<1 && right_boundary_distane != 0){
          roll2 = 0.15;
        }
        throttle2 = 1.1-dji_altitude;         
        throttle2 = speedLimit(-0.3,throttle2,0.3);
        controlVelYawRate.axes.push_back(0.3);     //pitch: forward --> positive
        controlVelYawRate.axes.push_back(roll2);     //roll:  left    --> positive
        controlVelYawRate.axes.push_back(throttle2);     //throttle: up
        controlVelYawRate.axes.push_back(0);     //yaw
        controlVelYawRate.axes.push_back(flag);
        ctrlFlightPub.publish(controlVelYawRate);
        ros::Duration(0.01).sleep();

        // ROS_INFO("cross control_gogogo \n");
        // ros::spinOnce();
      }
      if(ros::Time::now() - cross_time > ros::Duration(4)){ 
        tag_fly_count = 9;
        cross_time = ros::Time::now();
      }
      }
      case 10:{
        ROS_INFO("case1010101010101011010101010\n");
       while (ros::Time::now() - cross_time < ros::Duration(5))
      {
        pitch2 = 0;
        if(forward_last_distance<1.3 && forward_last_distance != 0){
          pitch2 = -0.1;
        }
        if(back_last_distance<1.3 && back_last_distance != 0){
          pitch2 = 0.1;
        }
        throttle2 = 1.1-dji_altitude;         
        throttle2 = speedLimit(-0.3,throttle2,0.3);
        controlVelYawRate.axes.push_back(0);     //pitch: forward --> positive
        controlVelYawRate.axes.push_back(-0.3);     //roll:  left    --> positive
        controlVelYawRate.axes.push_back(throttle2);     //throttle: up
        controlVelYawRate.axes.push_back(0);     //yaw
        controlVelYawRate.axes.push_back(flag);
        ctrlFlightPub.publish(controlVelYawRate);
        ros::Duration(0.01).sleep();

        ROS_INFO("cross control_gogogo \n");
        // ros::spinOnce();
      }
      if(ros::Time::now() - cross_time > ros::Duration(5)){ 
        controlVelYawRate.axes.clear();                //Very Important!!!!
        M100monitoredLand();
        ROS_INFO("landing!!!!!-------------___!!!!!!!!!!!!!!!!!!!----");
      }
      }
    }
////////////////////////////////switch end
  }
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
      throttle2 = 1.3-dji_altitude;         
      throttle2 = speedLimit(-0.3,throttle2,0.3);
      controlVelYawRate.axes.push_back(0.4);     //pitch: forward --> positive
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

