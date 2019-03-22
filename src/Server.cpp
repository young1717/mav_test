/*======================================================================*/

/**
 * @file
 *   @brief The serial interface process
 *
 *   This process connects any external MAVLink UART device to ROS
 *
 *   @author Lorenz Meier, <mavteam@student.ethz.ch>
 *
 */

#include "ros/ros.h"
#include "mavlink.h"

#include <glib.h>

#include <signal.h>

// udp libs
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// cv libs
#include <chrono>
#include <condition_variable>
#include <boost/signals2.hpp>
#include <mutex>

// Standard includes
#include <atomic> 
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <fstream>

#include <stdlib.h>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

#include "mav_test/Global_Position_INT.h"


#define BUFFER_LENGTH 2048 // minimum buffer size that can be used with qnx
#define LISTEN_PORT  2030

#define NETID 4		//自己的IP

using namespace std;
using std::string;

namespace sig2 = boost::signals2;
typedef sig2::signal<void(uint8_t *buf, ssize_t recsize, int ipflag)> MessageSig;
MessageSig udp_sig;

typedef std::lock_guard<std::recursive_mutex> lock_gd;

//ros::Publisher leader_global_pos_pub;
//ros::Publisher leader_sync_pub;
//ros::Publisher neighbour_state_pub;

// 局域网位置消息处理函数
void handle_message(uint8_t *buf, ssize_t recsize, int ipflag)
{
	if(ipflag==NETID)
		return;
		
	int i;
	//uint32_t ch;
	uint8_t msg_Received = false;

	// Something received - print out all bytes and parse packet
	mavlink_message_t message;
	mavlink_status_t status;

	for (i = 0; i < recsize; i++)
	{
		//ch = buf[i];
		msg_Received = mavlink_parse_char(MAVLINK_COMM_1, buf[i], &message, &status);
		if (status.packet_rx_drop_count != 0)
		{
			ROS_WARN("ERROR: DROPPED PACKETS");
		}
		// Packet received
		if (msg_Received)
		{
			switch (message.msgid)
			{
				/*
				 * Message specs (xxx: soon mavlink.org):
				 * https://pixhawk.ethz.ch/mavlink/#ATTITUDE
				 */

				case 0:
				{
					mavlink_heartbeat_t hb;
					mavlink_msg_heartbeat_decode(&message, &hb);
					
					// build state message after updating uas//需要在响应的msg文件中添加States.msg
					/*geese_formation::Sync sync_msg;
					sync_msg.signal = hb.base_mode;
					sync_msg.uas_num = ipflag;
					
					leader_sync_pub.publish(sync_msg);*/

					/*if(ipflag == 1)
					{
						leader_sync_pub.publish(sync_msg);
					}*/

				}
				break;	 
				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					mavlink_global_position_int_t gpos;
					mavlink_msg_global_position_int_decode(&message, &gpos);
					
					//可以将以上信息合并，计算出包含经、纬、高、星数、航向、vx,vy,vz的消息包
					mav_test::Global_Position_INT nei_gpos;
					
					nei_gpos.ip = ipflag;
					nei_gpos.time_boot_ms = gpos.time_boot_ms;

					nei_gpos.lat = gpos.lat;		// deg*1e7
					nei_gpos.lon = gpos.lon;		// deg*1e7
					nei_gpos.alt = gpos.alt;		// mm
				
					nei_gpos.vx = gpos.vx / 1e2;		// m/s
					nei_gpos.vy = gpos.vy / 1e2;		// m/s
					nei_gpos.vz = gpos.vz / 1e2;		// m/s
				
					nei_gpos.relative_alt = gpos.relative_alt / 1e3;		// m 相对高度
					nei_gpos.hdg = (gpos.hdg != UINT16_MAX) ? gpos.hdg / 1e2 : NAN;// in degrees

					/*if(ipflag == 1)
					{
						leader_global_pos_pub.publish(fix);
					}*/
					//if(ipflag != NETID)
					//	neighbour_state_pub.publish(fix);
				}
				break;
			}
		}
	}
}

void* udp_wait(void* ptr)
{
	//建立服务端套接字,若失败,进行提示;
	int srvSock = socket(AF_INET, SOCK_DGRAM, 0);
	if (srvSock == -1)
	{
		ROS_WARN("Socket Establishment Failed!");
	}
	//声明客户端和服务端的地址;并对服务端的IP和端口进行了定义;
	struct sockaddr_in srvaddr, clientaddr;
	bzero(&srvaddr, sizeof(srvaddr));
	srvaddr.sin_family = AF_INET;
	srvaddr.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("192.168.101.4");
	srvaddr.sin_port = htons(LISTEN_PORT);
	//将服务端套接字绑定在定义的地址上,即对任意ip的2030端口进行监听
	if (bind(srvSock, (struct sockaddr *)&srvaddr, sizeof(srvaddr)) == -1)
	{
		ROS_WARN("Bind Error!");
	}

	socklen_t addr_len = sizeof(struct sockaddr_in);  //接收地址的大小

	udp_sig.connect(handle_message);
	//--------------------------------------------------------//
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;

	char IP1[16] = "192.168.101.1";
	char IP2[16] = "192.168.101.2";
	char IP3[16] = "192.168.101.3";
	char IP4[16] = "192.168.101.4";
	char IP5[16] = "192.168.101.5";
	char IP6[16] = "192.168.101.6";
	char IP7[16] = "192.168.101.7";
	char IP8[16] = "192.168.101.8";
	char IP9[16] = "192.168.101.9";

	int ipflag;
	while (1)
	{
		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(srvSock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&clientaddr, &addr_len);
		if(clientaddr.sin_addr.s_addr == inet_addr(IP1)) ipflag = 1;
		if(clientaddr.sin_addr.s_addr == inet_addr(IP2)) ipflag = 2;
		if(clientaddr.sin_addr.s_addr == inet_addr(IP3)) ipflag = 3;
		if(clientaddr.sin_addr.s_addr == inet_addr(IP4)) ipflag = 4;
		if(clientaddr.sin_addr.s_addr == inet_addr(IP5)) ipflag = 5;
		if(clientaddr.sin_addr.s_addr == inet_addr(IP6)) ipflag = 6;
		if(clientaddr.sin_addr.s_addr == inet_addr(IP7)) ipflag = 7;
		if(clientaddr.sin_addr.s_addr == inet_addr(IP8)) ipflag = 8;
		if(clientaddr.sin_addr.s_addr == inet_addr(IP9)) ipflag = 9;

		if (recsize > 0)
		{
			udp_sig(buf, recsize, ipflag);
		}

	}
	return NULL;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Server");

	// SETUP ROS
	ros::NodeHandle nh;
	
	//neighbour_state_pub = nh.advertise<geese_formation::UAV_state>("neighbour_state", 100);
	
	//这部分程序是建立串口监听程序,一直接收串口数据,并进行处理
	int* ptr = NULL;
	GThread* udp_thread;
	udp_thread=g_thread_new("", udp_wait, (void *)ptr);
	
	ros::spin();

	return 0;
}
