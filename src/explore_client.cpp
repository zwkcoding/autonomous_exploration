#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iv_explore_msgs/ExploreAction.h>
#include <iv_explore_msgs/ObjectRecognitionStatus.h>

using namespace iv_explore_msgs;

bool complete_flag = false;
bool object_check_status = false;
// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
			const ExploreResultConstPtr& result)
{
	ROS_WARN("Explore actionclient:Finished in state [%s]", state.toString().c_str());
	ROS_INFO("Explore actionclient Answer: ");
	std::cout << std::boolalpha << result->success_flag << '\n';
	complete_flag = true;
}

// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("Explore actionclient: Goal just went active");
}

void objectCb(const iv_explore_msgs::ObjectRecognitionStatusConstPtr &msg) {
    if(0 != msg->object_search_result) {
        object_check_status = true;
    } else {
        object_check_status = false;

    }

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "explore_action_client");
	ros::NodeHandle n;
    ros::NodeHandle nh("~");

    iv_explore_msgs::ExploreGoal goal;
    iv_explore_msgs::ExploreResultConstPtr feedback;
	bool success;
	char ch;

    std::string re_object_send_topic_name;
    nh.param<std::string>("receive_object_send_topic_name", re_object_send_topic_name, "/object_status");
	actionlib::SimpleActionClient<iv_explore_msgs::ExploreAction> ac("explore", true);
    ros::Subscriber object_sub = n.subscribe(re_object_send_topic_name, 5, objectCb);

    ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();

	ROS_INFO("Action server started, sending goal.");
	
	do {

        ros::spinOnce();
        if(true == object_check_status) {
            ROS_WARN("Object Already Checked! awaiting for command input!");
            std::cin>>ch;
            if(ch =='q') {
                break;
            }
        }
        ROS_INFO("Waiting 2 seconds before sending goal.");
        // sleep 0.01 [sec]
        sleep(2);

//		ac.sendGoal(goal, &doneCb, &activeCb);

//		while(!complete_flag) {
//			ros::spinOnce();
//		}


//        complete_flag = false;
        ac.sendGoal(goal);
        // block here
        success = ac.waitForResult(ros::Duration(15.0));
        if (success) {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_WARN("Action finished: %s",state.toString().c_str());
        } else {
            ROS_WARN("Action did not finish before the time out. CANCEL goal");
            ac.cancelGoal();

        }

//        feedback = ac.getResult();
        /*std::cin>>ch;
        if(ch=='c') {
            ac.cancelGoal();
            break;
        }*/
	} while(1/*ch!='q'*/);

	return 0;
}
