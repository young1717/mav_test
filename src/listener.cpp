#include <unistd.h>

#include "ros/ros.h"
#include "mavlink.h"
//topic type
#include "mavros_msgs/State.h"
//service type
#include "mavros_msgs/SetMode.h"		///mavros/set_mode
#include "mavros_msgs/CommandBool.h"	///mavros/cmd/arming
#include "mavros_msgs/CommandTOL.h"		///mavros/cmd/takeoff
#include "mavros_msgs/CommandHome.h"	///mavros/cmd/set_home
#include "mavros_msgs/CommandLong.h"	///mavros/command(_long)
#include "mavros_msgs/CommandInt.h"		///mavros/command_int
#include "mavros_msgs/StreamRate.h"		///mavros/set_stream_rate

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
bool is_guided = false;
void chatterCallback(const mavros_msgs::State& msg)
{
	is_guided = msg.guided;
	ROS_INFO("I heard: [%d]", msg.guided);
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
	ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
	ros::Subscriber sub = n.subscribe("/mavros/state", 1000, chatterCallback);

	ros::ServiceClient mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	mavros_msgs::SetMode mode_srv;
	mode_srv.request.base_mode = 0;
	mode_srv.request.custom_mode = "GUIDED";
	if(mode_client.call(mode_srv)){
		sleep(1);
		if(is_guided)
			ROS_INFO("Change the mode to GUIDED successfully!");
	}

//	ros::ServiceClient arm_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
//	mavros_msgs::CommandBool arm_srv;
//	arm_srv.request.value = true;
//	if(arm_client.call(arm_srv)){
//		ROS_INFO("Arm succeed!");
//	}
	
//	mavlink_message_t message;
//	mavlink_status_t status;
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
	ros::spin();

	return 0;
}
