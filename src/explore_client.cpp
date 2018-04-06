#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iv_explore_msgs/ExploreAction.h>

using namespace iv_explore_msgs;

bool complete_flag = false;
// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
			const ExploreResultConstPtr& result)
{
	ROS_INFO("Explore actionclient:Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Explore actionclient Answer: ");
	std::cout << std::boolalpha << result->success_flag << '\n';
	complete_flag = true;
}

// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("Explore actionclient: Goal just went active");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "explore_action_client");
	ros::NodeHandle n;

    iv_explore_msgs::ExploreGoal goal;
    iv_explore_msgs::ExploreResultConstPtr feedback;
	bool success;
	char ch;

	actionlib::SimpleActionClient<iv_explore_msgs::ExploreAction> ac("explore", true);
	
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();

	ROS_INFO("Action server started, sending goal.");
	
	do {

		ac.sendGoal(goal, &doneCb, &activeCb);
//		success = ac.waitForResult(ros::Duration(10.0));

		while(!complete_flag) {
			ros::spinOnce();
		}


        complete_flag = false;
//        actionlib::SimpleClientGoalState state = ac.getState();
//        ROS_INFO("Action finished: %s", state.toString().c_str());
        feedback = ac.getResult();

        /*std::cin>>ch;
        if(ch=='c') {
            ac.cancelGoal();
            break;
        }*/
	} while(1/*ch!='q'*/);

	return 0;
}
