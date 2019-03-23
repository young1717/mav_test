#pragma once
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

#include <map>
#include <vector>

#include "mav_test/Global_Position_INT.h"
#include "sensor_msgs/NavSatFix.h"

#define BUFFER_LENGTH 2048 // minimum buffer size that can be used with qnx
#define LISTEN_PORT  2030
#define BROADCAST_IP "192.168.101.255" // LOCAL_BROADCAST
#define NETID 4		//自己的IP

// Task State Machine Definitions
#define TASK_NONE 0
//#define TASK_WAIT_TAKEOFF_SIGNAL 1
#define TASK_DO_ARMING 1
#define TASK_DO_DISARMING 2
#define TASK_DO_SET_GUIDED_MODE 3
#define TASK_DO_TAKEOFF 4
#define TASK_WAIT_FOR_STABLE 5
#define TASK_DO_NAV_CONTROL 6
#define TASK_DO_LAND 7
#define TASK_FINISH 8
#define TASK_DO_SET_HOME 9

using namespace std;
using std::string;

namespace sig2 = boost::signals2;
typedef sig2::signal<void(uint8_t *buf, ssize_t recsize, int ipflag)> MessageSig;
typedef std::lock_guard<std::recursive_mutex> lock_gd;

class UAV
{
public:
	UAV(){}
	void update_uav_global_pos(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void update_nei_global_pos(const mav_test::Global_Position_INT &msg, uint16_t flag);
	mav_test::Global_Position_INT get_neighbours_state(uint16_t flag);
	mav_test::Global_Position_INT get_neighbours_state2(uint16_t flag);
	uint16_t get_neighbours_num();
	uint16_t get_neighbours_flag();
	void remove_neighbours(void);//call at a certain frequency

private:
	std::recursive_mutex mutx;
	
	mav_test::Global_Position_INT self_global_pos;

	std::map<uint16_t, mav_test::Global_Position_INT> neighbours_global_pos;
	uint16_t neighbours_flag;
	uint16_t last_neighbours_flag;
};


