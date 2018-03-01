#include <ros/ros.h>
#include <map>
#include <queue>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <opt_utils/opt_utils.hpp>

#include <autonomous_exploration/ExploreAction.h>
#include <autonomous_exploration/GridMap.h>
#include <autonomous_exploration/VisMarker.h>
#include <autonomous_exploration/PoseWrap.h>
#include <autonomous_exploration/Color.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace tf;
using namespace ros;

class ExploreAction {

public:

   	ExploreAction(std::string name) : 
		as_(nh_, name, boost::bind(&ExploreAction::run, this, _1), false),
		action_name_(name),
		ac_("move_base", true)
	{
		mMarkerPub_ = nh_.advertise<visualization_msgs::Marker>("vis_marker", 2);
		mGetMapClient_ = nh_.serviceClient<nav_msgs::GetMap>(std::string("current_map"));

		double laser_range;
		nh_.param("laser_max_range", laser_range, 2.);
		mCurrentMap_.setLaserRange(laser_range);

		int lethal_cost;
		nh_.param("lethal_cost", lethal_cost, 100);
		mCurrentMap_.setLethalCost(lethal_cost);

		nh_.param("gain_threshold", initial_gain_, 10.);
		mCurrentMap_.setGainConst(initial_gain_);
		
		double robot_radius;
		nh_.param("robot_radius", robot_radius, 0.19);
		mCurrentMap_.setRobotRadius(robot_radius);

		std::string map_path;
		std::string temp = "/";
		nh_.param("map_path", map_path, temp);
		mCurrentMap_.setPath(map_path);

		nh_.param("min_distance", minDistance_, 1.);
		nh_.param("min_gain_threshold", minGain_, 2.);
		nh_.param("gain_change", gainChangeFactor_, 1.5);

		// todo zwk
		// keep initial pose
		double xinit_ = 0, yinit_ = 0;
		TransformListener mTfListener_;
		try {
			std::string world_frame_id_ = "/odom";
			Time now = Time::now();
			mTfListener_.waitForTransform(world_frame_id_, std::string("/base_link"), Time(0), Duration(10.0));

			tf::StampedTransform transform;
			mTfListener_.lookupTransform(world_frame_id_, std::string("/base_link"), Time(0), transform);

			 xinit_ = transform.getOrigin().x();
			 yinit_ = transform.getOrigin().y();
		}
		catch(TransformException ex)
		{
			ROS_ERROR("[Node: explore_server]: Could not get robot position: %s", ex.what());
		}
		mCurrentMap_.setInitPisition(xinit_, yinit_);

		as_.registerPreemptCallback(boost::bind(&ExploreAction::preemptCB, this));

		as_.start();

	}

	~ExploreAction(void) 
	{
	}

    void run(const autonomous_exploration::ExploreGoalConstPtr &goal)
	{
		Rate loop_rate(4);

		// zwk todo
//		double current_gain = mCurrentMap_.getGainConst();
		double current_gain = initial_gain_;

		int count = 0;
		double target_x, target_y;

		while(ok() && as_.isActive() && current_gain >= minGain_)
		{
			loop_rate.sleep();
			mCurrentMap_.setGainConst(current_gain);
			
			unsigned int pos_index;

			if(!getMap()) 
			{
				ROS_ERROR("Could not get a map");
				as_.setPreempted();
				return;
			}

			if(!mCurrentMap_.getCurrentPosition(pos_index))
			{
				ROS_ERROR("Could not get a position");
				as_.setPreempted();
				return;
			}

			auto start = hmpl::now();
			int explore_target = explore(&mCurrentMap_, pos_index);
			auto end = hmpl::now();
			std::cout << "once explore cost time:" << hmpl::getDurationInSecs(start, end) << "\n";

			if(explore_target != -1) 
			{
				// add by zwk

				mCurrentMap_.getOdomCoordinates(target_x, target_y, explore_target);

				ROS_INFO("Got the explore goal: %f, %f\n", target_x, target_y);
				ROS_INFO("adjust info gain numbers and cur info: %d, %f\n", count, current_gain);

				break;

				moveTo(explore_target);

				if(count) 
				{
					current_gain *= gainChangeFactor_;
					count--;
					ROS_INFO("Gain was increased to: %f", current_gain);
				}
			}
			else
			{
				current_gain /= gainChangeFactor_;
				count++;
				ROS_INFO("Gain was decreased to: %f", current_gain);
			}
		}

		if(as_.isActive())
		{
			if( current_gain >= minGain_) {
				result_.target.x = target_x ;
				result_.target.y = target_y ;
				as_.setSucceeded(result_);
				ROS_INFO("Exploration finished");
			} else {
				as_.setAborted();
				ROS_INFO("Exploration failed : current info_gain is less than min");
			}
		}
		else
		{
			as_.setAborted();
			ROS_INFO("Exploration was interrupted");
		}
    }

	void preemptCB() 
	{
		ROS_INFO("Server received a cancel request.");
//		mCurrentMap_.generateMap();
		goalReached_ = -1;
		ac_.cancelGoal();
		as_.setPreempted();
	}    
	
	void doneCb(const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResult::ConstPtr& result)
    {
		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Robot reached the explore target");
			goalReached_ = 1;
		} 
		else
		{
			goalReached_ = -1;
		}
    }

private:
    int explore(GridMap *map, unsigned int start)
    {
	    ROS_INFO("Starting exploration");

		map->clearArea(start);
		ROS_INFO("Cleared the area");

	    unsigned int mapSize = map->getSize();
		// represent visit flag table
	    double *plan = new double[mapSize];
	    for(unsigned int i = 0; i<mapSize; i++)
	    {
		    plan[i] = -1;
	    }

	    Queue queue;
	    Entry startPoint(0.0, start);
	    queue.insert(startPoint);
	    plan[start] = 0;

	    Queue::iterator next;
	    double distance;
	    double linear = map->getResolution();
	    int foundFrontier = 0;

	    while(!queue.empty() && !foundFrontier && goalReached_ != 2) 
	    {
		    next = queue.begin();
		    distance = next->first;
		    unsigned int index = next->second;
		    queue.erase(next);

			ROS_DEBUG("Target distance: %f", distance);
			//todo zwk improve it
		    if(distance > minDistance_ && map->isFrontier(index))
		    {
				foundFrontier = index;
				publishMarker(index, 2);
		    }
		    else
		    {
			    unsigned int ind[4];

			    ind[0] = index - 1;
			    ind[1] = index + 1;
			    ind[2] = index - map->getWidth();
			    ind[3] = index + map->getWidth();

			    for(unsigned int it = 0; it<4; ++it)
			    {
				    unsigned int i = ind[it];
				    if(map->isFree(i) && plan[i] == -1) 
				    {
					    queue.insert(Entry(distance+linear, i));
					    plan[i] = distance+linear;

						publishMarker(index, 1);
				    }
			    }
		    }
	    }

	    delete [] plan;
		queue.clear();

	    if(foundFrontier) 
	    {
		    return foundFrontier;
	    }
	    else
	    {
			return -1;
	    }
    }

    bool moveTo(unsigned int goal_index)
    {

		// add by zwk todo
		ROS_INFO("Failed to reach the goal");
		return false;

		Rate loop_rate(8);

		while(!ac_.waitForServer()) 
		{
			ROS_INFO("Waiting for the move_base action server to come up");
			loop_rate.sleep();
		}

		double x, y;
		mCurrentMap_.getOdomCoordinates(x, y, goal_index);
	
		ROS_INFO("Got the explore goal: %f, %f\n", x, y);

	   	move_base_msgs::MoveBaseGoal move_goal;

		move_goal.target_pose.header.frame_id = "local_map/local_map";
		move_goal.target_pose.header.stamp = Time(0);

		PoseWrap pose(x, y);
		move_goal.target_pose.pose = pose.getPose();
		
		ROS_INFO("Sending goal");
		ac_.sendGoal(move_goal,
			     boost::bind(&ExploreAction::doneCb, this, _1, _2));
		
		goalReached_ = 0;

		while(goalReached_ == 0 && ok()) 
		{
			loop_rate.sleep();
		}
		
		if(goalReached_ == 1) 
		{
			ROS_INFO("Reached the goal");
			return true;
		}   
		
		ROS_INFO("Failed to reach the goal");	
		return false;
    }

    bool getMap() 
    {
	    if(!mGetMapClient_.isValid()) 
		{
		    return false;
		}

	    nav_msgs::GetMap srv;

	    if(!mGetMapClient_.call(srv)) 
	    {
		    ROS_INFO("Could not get a map.");
		    return false;
	    }

	    mCurrentMap_.update(srv.response.map);
	    ROS_INFO("Got new map of size %d x %d", mCurrentMap_.getWidth(), mCurrentMap_.getHeight());
	    ROS_INFO("Map resolution is: %f", mCurrentMap_.getResolution());

	    return true;
    }

	void publishMarker(unsigned int index, int type) {
		double x, y;
		mCurrentMap_.getOdomCoordinates(x, y, index);
		PoseWrap pose(x, y);

		VisMarker marker;
		/*
		  type:
			1: free area
			2: frontier
		*/
		if(type == 1) {
			Color color(0, 0, 0.7);
			marker.setParams("free", pose.getPose(), 0.35, color.getColor());
		} else
		if(type == 2) {
			Color color(0, 0.7, 0);
			marker.setParams("frontier", pose.getPose(), 0.75, color.getColor());
		}

		mMarkerPub_.publish(marker.getMarker());
	}	

protected:
    NodeHandle nh_;
	actionlib::SimpleActionServer<autonomous_exploration::ExploreAction> as_;
	autonomous_exploration::ExploreFeedback feedback_;
	autonomous_exploration::ExploreResult result_;
	std::string action_name_;
	MoveBaseClient ac_;

    ServiceClient mGetMapClient_;
    GridMap mCurrentMap_;
	double minDistance_;
	double minGain_;
	double initial_gain_;
	double gainChangeFactor_;
    int goalReached_;

	Publisher mMarkerPub_;
};

int main(int argc, char **argv) 
{
	init(argc, argv, "explore_server_node");

	ExploreAction explore("explore");
	
	spin();
	
	return 0;
}
