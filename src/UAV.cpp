#include "Client.h"

void UAV::update_uav_global_pos(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	lock_gd lock(mutx);
	self_global_pos.lat = msg->latitude;
	self_global_pos.lon = msg->longitude;
	self_global_pos.alt = msg->altitude;
}


void UAV::update_nei_global_pos(const mav_test::Global_Position_INT &msg, uint16_t flag)
{
	lock_gd lock(mutx);

	auto iter = neighbours_global_pos.find(flag);
	if(iter == neighbours_global_pos.end())
		neighbours_global_pos.insert(std::pair<uint16_t, mav_test::Global_Position_INT>(flag, msg));
	else if(iter != neighbours_global_pos.end())
		iter->second = msg;
	neighbours_flag |= 1<<flag;
}

mav_test::Global_Position_INT UAV::get_neighbours_state(uint16_t flag)
{
	lock_gd lock(mutx);
	auto iter = neighbours_global_pos.find(flag);
	if(iter != neighbours_global_pos.end())
		return iter->second;
}

mav_test::Global_Position_INT UAV::get_neighbours_state2(uint16_t flag)
{
	lock_gd lock(mutx);
	auto iter = neighbours_global_pos.begin();
	for(int i = 0; i < flag; i++)
		iter++;
	if(iter != neighbours_global_pos.end())
		return iter->second;
}

uint16_t UAV::get_neighbours_num()
{
	lock_gd lock(mutx);
	return neighbours_global_pos.size();
}

uint16_t UAV::get_neighbours_flag()
{
	lock_gd lock(mutx);
	return last_neighbours_flag;
}

void UAV::remove_neighbours(void)
{
	lock_gd lock(mutx);
	uint16_t diff_flag = last_neighbours_flag^neighbours_flag;
	for(int i = 0; i < 10; i++)
	{
		if((diff_flag&(1<<i))&&(!(neighbours_flag&(1<<i))))
		{
			neighbours_global_pos.erase(i);
		}
	}
	last_neighbours_flag = neighbours_flag;
	neighbours_flag = 0;
	
	//cout << bitset<sizeof(uint16_t)*8>(last_neighbours_flag) << endl;
}


