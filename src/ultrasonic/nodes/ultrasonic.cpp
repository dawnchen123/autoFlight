
#include <ros/ros.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>

#include <stdio.h>
#include <string.h>
#include "serial.h"
#include "crc32.h"
#include "protocal_uart_sdk.h"
#include "DJI_guidance.h"

#define UART 1
#define CAMERA_PAIR_NUM 5
#define UART_PORT 1

std_msgs::Float32 heightSonar;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ultrasonic");
  ros::NodeHandle nh;
  ros::Publisher heightPub;
  ros::Rate loop_rate(50.0);
  heightPub = nh.advertise<std_msgs::Float32>("ultraSonic/height", 10);

  	if ( connect_serial( UART_PORT ) < 0 )
	{
        printf( "connect serial error\n" );
        return 0;
	}
	for (;;)
	// for ( int i = 0; i < 1000; ++i )
	{
		unsigned char data[1000] = {0};
		int max_size = (int)sizeof(data);
		int timeout = 0.05;
		int n = read_serial( data, max_size, timeout);
		if( n <= 0 )
		{
			continue;
		}

		push( data, sizeof(data) );
		while(ros::ok())
		{
			//printf("this is ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;:\n");
			unsigned int len = 0;
			int has_packet = pop( data, len );
			if ( has_packet )
			{
				if ( len )
				{
					unsigned char cmd_id = data[1];
					// if ( e_imu == cmd_id )
					// {
					// 	imu imu_data;
					// 	memcpy( &imu_data, data + 2, sizeof(imu_data) );
					// 	printf( "imu:%f %f %f,%f %f %f %f\n", imu_data.acc_x, imu_data.acc_y, imu_data.acc_z, 
					// 		     imu_data.q[0], imu_data.q[1], imu_data.q[2], imu_data.q[3] );
					// 	printf( "frame index:%d,stamp:%d\n", imu_data.frame_index, imu_data.time_stamp );
					// 	printf( "\n" );
					// }
					if ( e_ultrasonic == cmd_id )
					{
						ultrasonic_data ultrasonic;
						memcpy( &ultrasonic, data + 2, sizeof(ultrasonic) );
						for ( int d = 0; d < CAMERA_PAIR_NUM; ++d )
						{

							heightSonar.data = ultrasonic.ultrasonic[0] * 0.001f;
						        // printf( "distance:%f,reliability:%d\n", ultrasonic.ultrasonic[0] * 0.001f, (int)ultrasonic.reliability[d] );
						}
						// printf( "frame index:%d,stamp:%d\n", ultrasonic.frame_index, ultrasonic.time_stamp );
						// printf( "\n" );
						// heightPub.publish(heightSonar);
					}
					// if ( e_velocity == cmd_id )
					// {
					// 	velocity vo;
					// 	soc2pc_vo_can_output output;
					// 	memcpy( &output, data + 2, sizeof(vo) );
					// 	vo.vx = output.m_vo_output.vx;
					// 	vo.vy = output.m_vo_output.vy;
					// 	vo.vz = output.m_vo_output.vz;
					// 	printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo.vx, 0.001f * vo.vy, 0.001f * vo.vz );
					// 	printf( "frame index:%d,stamp:%d\n", vo.frame_index, vo.time_stamp );
					// 	printf( "\n" );
					// }
					// if ( e_obstacle_distance == cmd_id )
					// {
					// 	obstacle_distance oa;
					// 	memcpy( &oa, data + 2, sizeof(oa) );
					// 	printf( "obstacle distance:" );
					// 	for ( int direction = 0; direction < CAMERA_PAIR_NUM; ++direction )
					// 	{
					// 		printf( " %f ", 0.01f * oa.distance[direction] );
					// 	}
					// 	printf( "\n" );
					// 	printf( "frame index:%d,stamp:%d\n", oa.frame_index, oa.time_stamp );
					// 	printf( "\n" );
					// }
				}
				else
				{
					printf( "ultraSonic err\n" );
				}
			}
			else
			{
				break;
			}
			// loop_rate.sleep();
			if(heightSonar.data>0.01 && heightSonar.data<3) {
				heightPub.publish(heightSonar);
			}

		}
	}

	disconnect_serial();
  return EXIT_SUCCESS;
}
