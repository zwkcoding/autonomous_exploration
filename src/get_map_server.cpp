#include <ros/ros.h>
#include <map>
#include <std_msgs/Float64.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <boost/thread.hpp>

using namespace ros;

nav_msgs::GetMap::Response global_map_;
nav_msgs::GetMap::Response cover_map_;
boost::mutex global_map_mutex, cover_map_mutex;
bool got_global_map, got_cover_map;
int g_obstacle_threshold;
// todo could make changes on map(ROI, replace value...)here
void globalMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	ROS_DEBUG("Updating a global map");
	boost::mutex::scoped_lock map_lock (global_map_mutex);

	global_map_.map.info.width = map->info.width;
	global_map_.map.info.height = map->info.height;
    global_map_.map.info.resolution = map->info.resolution;

	global_map_.map.info.map_load_time = map->info.map_load_time;

	global_map_.map.info.origin.position.x = map->info.origin.position.x;
    global_map_.map.info.origin.position.y = map->info.origin.position.y;
	global_map_.map.info.origin.position.z = map->info.origin.position.z;

    global_map_.map.info.origin.orientation.x = map->info.origin.orientation.x;
    global_map_.map.info.origin.orientation.y = map->info.origin.orientation.y;
	global_map_.map.info.origin.orientation.z = map->info.origin.orientation.z;
    global_map_.map.info.origin.orientation.w = map->info.origin.orientation.w;

	global_map_.map.data.resize(map->info.width*map->info.height);

//	std::map<int, int> mp;
    /**
     * binary_map_ ： 100 --> OBSTACLE
     *               0 --> FREE/UNKNOWN
     * triple_map_ : 0 --> FREE
     *               100 --> OBSTACLE
     *               -1 --> UNKNOWN
     */
	for(int i=0; i<map->data.size(); i++) {
//        mp[map->data[i]]++;
        if(map->data[i] == -1) {
            global_map_.map.data[i] = -1;
		} else if(map->data[i] <= g_obstacle_threshold) {
            global_map_.map.data[i] = 0;
		} else {
            global_map_.map.data[i] = 100;
		}
    }

	global_map_.map.header.stamp = map->header.stamp;
    global_map_.map.header.frame_id = map->header.frame_id;
/*
	ROS_INFO("Got map!");
	for(std::map<int, int>::iterator it=mp.begin(); it!=mp.end(); it++)
		ROS_INFO("%d, %d", it->first, it->second);
*/
	got_global_map = true;
}

void coverMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    ROS_DEBUG("Updating a cover map");
    boost::mutex::scoped_lock map_lock (cover_map_mutex);

    cover_map_.map.info.width = map->info.width;
    cover_map_.map.info.height = map->info.height;
    cover_map_.map.info.resolution =  map->info.resolution;

    cover_map_.map.info.map_load_time =  map->info.map_load_time;

    cover_map_.map.info.origin.position.x = map->info.origin.position.x;
    cover_map_.map.info.origin.position.y = map->info.origin.position.y;
    cover_map_.map.info.origin.position.z = map->info.origin.position.z;

    cover_map_.map.info.origin.orientation.x = map->info.origin.orientation.x;
    cover_map_.map.info.origin.orientation.y = map->info.origin.orientation.y;
    cover_map_.map.info.origin.orientation.z = map->info.origin.orientation.z;
    cover_map_.map.info.origin.orientation.w = map->info.origin.orientation.w;

    cover_map_.map.data.resize(map->info.width*map->info.height);

//    std::map<int, int> mp;
    /**
     * binary_map_ ： 100 --> OBSTACLE
     *               0 --> FREE/UNKNOWN
     * triple_map_ : 0 --> FREE
     *               100 --> OBSTACLE
     *               -1 --> UNKNOWN
     */
    for(int i=0; i<map->data.size(); i++) {
//        mp[map->data[i]]++;
        if(map->data[i] == -1) {
            cover_map_.map.data[i] = -1;
        } else if(map->data[i] <= g_obstacle_threshold) {
            cover_map_.map.data[i] = 0;
        } else {
            cover_map_.map.data[i] = 100;

        }
    }

    cover_map_.map.header.stamp = map->header.stamp;
    cover_map_.map.header.frame_id = map->header.frame_id;
/*
	ROS_INFO("Got map!");
	for(std::map<int, int>::iterator it=mp.begin(); it!=mp.end(); it++)
		ROS_INFO("%d, %d", it->first, it->second);
*/
    got_cover_map = true;
}


bool mapCallback(nav_msgs::GetMap::Request  &req, 
		 nav_msgs::GetMap::Response &res)
{
	boost::mutex::scoped_lock map_lock (global_map_mutex);
	if(got_global_map && global_map_.map.info.width && global_map_.map.info.height)
	{
		res = global_map_;
		return true;
	}
	else
		return false;
}

bool coverMapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
	boost::mutex::scoped_lock map_lock (cover_map_mutex);
	if(got_cover_map && cover_map_.map.info.width && cover_map_.map.info.height)
	{
		res = cover_map_;
		return true;
	}
	else
		return false;
}


int main(int argc, char **argv)
{
	init(argc, argv, "map_server");
	NodeHandle n;
    ros::NodeHandle private_nh_("~");
    int obstacle_value = 0;
	std::string global_map_topic_name, cover_map_topic_name;
    private_nh_.param<int>("obs_threshold", obstacle_value, 80);
	private_nh_.param<std::string>("receive_global_map_topic_name", global_map_topic_name, "/global_map");
    private_nh_.param<std::string>("receive_cover_map_topic_name", cover_map_topic_name, "/cover_map");

    g_obstacle_threshold = obstacle_value;

	Subscriber global_map_sub = n.subscribe(global_map_topic_name, 10, globalMapUpdate);
    Subscriber cover_map_sub = n.subscribe(cover_map_topic_name, 10, coverMapUpdate);

	ServiceServer ss = n.advertiseService("global_map", mapCallback);

	ServiceServer sss = n.advertiseService("cover_map", coverMapCallback);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin(); // spin() will not return until the node has been shutdown
//	spin();

	return 0;
}
