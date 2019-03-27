#include "Client.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
UAV uav;

void chatterCallback(const mavros_msgs::StatePtr msg)
{
	uav.update_uav_state(msg);
}

void global_pos_callback(const sensor_msgs::NavSatFixPtr msg)
{
	uav.update_uav_global_pos(msg);
}

void vel_callback(const geometry_msgs::TwistStampedPtr msg)
{
	uav.update_uav_vel(msg);
}

void sub_position_state(void);

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
	thread th_(sub_position_state);

	mavros_msgs::State uav_state = uav.get_uav_state();

	ros::Publisher position_pub = n.advertise<mav_test::Global_Position_INT>("/mav_test/position_state", 1000);

	auto arm_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	mavros_msgs::CommandBool arm_srv;
	arm_srv.request.value = true;
	//if(arm_client.call(arm_srv)){
	//	ROS_INFO("Arm succeed!");
	//}

	auto mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
	mavros_msgs::SetMode mode_srv;
	mode_srv.request.base_mode = 0;
	mode_srv.request.custom_mode = "GUIDED";
	if(mode_client.call(mode_srv)){
		sleep(1);
		if(uav_state.guided)
			ROS_INFO("Change the mode to GUIDED successfully!");
	}
	
	auto home_client = n.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
	mavros_msgs::CommandHome home_srv;

	auto takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
	mavros_msgs::CommandTOL takeoff_srv;

	auto land_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
	mavros_msgs::CommandTOL land_srv;
	
	ros::Rate loop_rate(10);
	uint8_t task_state = 0, old_task_state = 255;
	while(ros::ok()){
		position_pub.publish(uav.get_global_pos());
		uav_state = uav.get_uav_state();
		switch(task_state){
			case TASK_DO_ARMING:
			{
				if(task_state!=old_task_state){
					ROS_INFO("TASK_DO_ARMING...");
					old_task_state = task_state;
				}
				if(uav_state.armed){
					task_state = TASK_DO_SET_GUIDED_MODE;
				}
				break;
			}
			case TASK_DO_DISARMING:
			{
				if(task_state!=old_task_state){
					ROS_INFO("TASK_DO_DISARMING");
					old_task_state = task_state;
				}
				if(uav_state.armed){
					arm_srv.request.value = false;
					if(arm_client.call(arm_srv))
						ROS_INFO("DisArm!");
					else
						ROS_WARN("DisArm didn't finish!");
				}else
					task_state = TASK_FINISH;
				break;
			}
			case TASK_DO_SET_GUIDED_MODE:
			{
				if(task_state!=old_task_state){
					ROS_INFO("TASK_DO_SET_GUIDED_MODE");
					old_task_state = task_state;
				}
				if(!uav_state.guided){
					mode_srv.request.custom_mode = "GUIDED";
					if(mode_client.call(mode_srv))
						ROS_INFO("Turn to GUIDED!");
					else
						ROS_WARN("GUIDED failed!");
				}else if(uav_state.guided&&uav_state.armed)
					task_state = TASK_DO_SET_HOME;
				else
					task_state = TASK_FINISH;
				break;
			}
			case TASK_DO_SET_HOME:
			{
				if(task_state!=old_task_state){
					ROS_INFO("TASK_DO_SET_HOME");
					old_task_state = task_state;
				}
				if(uav_state.armed && uav_state.guided)
				{
					home_srv.request.current_gps = true;
					if(home_client.call(home_srv)&&home_srv.response.success)
						task_state = TASK_DO_TAKEOFF;
				}
				break;
			}
			case TASK_DO_TAKEOFF:
			{
				if(task_state!=old_task_state){
					ROS_INFO("TASK_DO_TAKEOFF");
					old_task_state = task_state;
				}
				if(uav_state.armed && uav_state.guided)
				{
					if(takeoff_client.call(takeoff_srv)&&takeoff_srv.response.success)
						task_state = TASK_WAIT_FOR_STABLE;
				}
				break;
			}
			case TASK_WAIT_FOR_STABLE:
			{
				if(task_state!=old_task_state){
					ROS_INFO("TASK_WAIT_FOR_STABLE");
					old_task_state = task_state;
				}
				task_state = TASK_DO_NAV_CONTROL;
				break;
			}
			case TASK_DO_NAV_CONTROL:
			{
				if(task_state!=old_task_state){
					ROS_INFO("TASK_DO_NAV_CONTROL");
					old_task_state = task_state;
				}
				if(uav_state.armed && uav_state.guided)
				{
					//left_right_flag = formation();
					/*if(finish)
					{
						task_state = TASK_DO_LAND;
					}*/
				}else if(!uav_state.armed)
				{
					task_state = TASK_FINISH;
				}
				break;
			}
			case TASK_DO_LAND:
			{
				if(task_state!=old_task_state){
					ROS_INFO("TASK_DO_LAND");
					old_task_state = task_state;
				}
				if(uav_state.armed && uav_state.guided)
				{
					if(land_client.call(land_srv)&&land_srv.response.success)
						task_state = TASK_FINISH;
				}
				break;
			}
			case TASK_FINISH:
			{
				if(task_state!=old_task_state){
					ROS_INFO("TASK_FINISH");
					old_task_state = task_state;
				}
				if(!uav_state.armed)
					task_state = TASK_DO_ARMING;
				break;
			}
			default:
			{
				task_state = TASK_DO_ARMING;
			}
		}

		loop_rate.sleep();
	}
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
	ros::spin();

	return 0;
}


void sub_position_state()
{
	ROS_INFO("THREAD:sub_position_state");
	ros::NodeHandle nh;
	ros::Subscriber state_sub = nh.subscribe("/mavros/state", 1000, chatterCallback);
	ros::Subscriber global_sub = nh.subscribe("/mavros/global_position/raw/fix", 1000, global_pos_callback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/global_position/local", 1000, vel_callback);

	ros::spin();
}
