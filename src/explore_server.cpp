#include <ros/ros.h>
#include <map>
#include <queue>
#include <algorithm> // sort

#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <opt_utils/opt_utils.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <path_transform/path_planning.hpp>


#include "autonomous_exploration/bfs_frontier_search.hpp"
#include "autonomous_exploration/util.hpp"
#include "autonomous_exploration/Config.hpp"
#include <autonomous_exploration/ExploreAction.h>
#include <autonomous_exploration/GridMap.h>
#include <autonomous_exploration/VisMarker.h>
#include <autonomous_exploration/PoseWrap.h>
#include <autonomous_exploration/Color.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace tf;
using namespace ros;
using namespace frontier_exploration;

#define DEBUG


/**
     * Used to store data for each exploration point.
     * After all expl. points have been processed the overallValue
     * can be calculated which is used to sort the points.
     */
struct ExplorationPoint {

    ExplorationPoint(PoseWrap expl_pose, unsigned int num_expl_cells, double ang_dist, double robot_point_dist) : explPose(expl_pose),
                                                                   numberOfExploredCells(num_expl_cells),
                                                                   angularDistance(ang_dist),
                                                                   robotPointDistance(robot_point_dist),
                                                                    expl_value(0.0),
                                                                   ang_value(0.0), dist_value(0.0){
    }

    bool operator<(const ExplorationPoint &rhs) const {
        return overallValue < rhs.overallValue;
    }

    /**
     * Calculates the overall value of this exploratin point.
     * If max_explored_cells or max_robot_goal_dist is 0, the explored cells
     * or the the distance of the goal point is ignored.
     * The bigger the better.
     */
    double calculateOverallValue(config::Weights &weights, double max_explored_cells, double max_robot_goal_dist) {
        if (max_explored_cells != 0) {
            expl_value = weights.explCells * (numberOfExploredCells / max_explored_cells);
        }
        ang_value = weights.angDist * (1 - (angularDistance / M_PI));
        if (max_robot_goal_dist != 0) {
            dist_value = weights.robotGoalDist * (1 - (robotPointDistance / max_robot_goal_dist));
        }
        overallValue = expl_value + ang_value + dist_value;
        return overallValue;
    }

    double overallValue;
    PoseWrap explPose;
    unsigned int numberOfExploredCells;
    double angularDistance;
    double robotPointDistance;

    double expl_value;
    double ang_value;
    double dist_value;

};



class ExploreAction {

public:

    ExploreAction(std::string name, config::Config cfg) : as_(nh_, name, boost::bind(&ExploreAction::run, this, _1), false),
                                      ac_("move_base", true), config_(cfg){
//		mMarkerPub_ = nh_.advertise<visualization_msgs::Marker>("vis_marker", 2);
        mMarkerPub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis_marker", 2);
        binary_gom_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("binary_ogm", 2);
        mGetMapClient_ = nh_.serviceClient<nav_msgs::GetMap>(std::string("current_map"));
        mGetBinaryMapClient_ = nh_.serviceClient<nav_msgs::GetMap>(std::string("binary_map"));

        start_sub_ = nh_.subscribe("/initialpose", 1, &ExploreAction::startCb, this);
        path_publisher_ = nh_.advertise<nav_msgs::Path>("PT_path", 1, true);
        double laser_range = 8.0;
//		nh_.param("laser_max_range", laser_range, 8.);
        mCurrentMap_.setLaserRange(laser_range);

        int lethal_cost;
        nh_.param("lethal_cost", lethal_cost, 70);
        mCurrentMap_.setLethalCost(lethal_cost);

        nh_.param("gain_threshold", initial_gain_, 20.);
        mCurrentMap_.setGainConst(initial_gain_);

        double robot_radius = config_.vehicle_length / 2;
//		nh_.param("robot_radius", robot_radius, 2.);
        mCurrentMap_.setRobotRadius(robot_radius);

        std::string map_path;
        std::string temp = "/";
        nh_.param("map_path", map_path, temp);
        mCurrentMap_.setPath(map_path);

        minDistance_ = 5.0;
//		nh_.param("min_distance", minDistance_, 5.);
        nh_.param("min_gain_threshold", minGain_, 0.5);
        nh_.param("gain_change", gainChangeFactor_, 1.5);

        start_point_ = hmpl::Pose2D(10, 10, M_PI);

        // keep initial pose
        double xinit_ = 0, yinit_ = 0;
        tf::TransformListener mTfListener;
        tf::StampedTransform transform;
        std::string world_frame_id_ = "/odom";
        int temp0 = 0;
        // wait here until receive tf tree
        while (temp0 == 0) {
            try {
                temp0 = 1;
                mTfListener.lookupTransform(world_frame_id_, std::string("/base_link"), ros::Time(0), transform);
            } catch (tf::TransformException ex) {
                temp0 = 0;
                ros::Duration(0.1).sleep();
                ROS_INFO("no tf tree is received!");

            }
        }
        xinit_ = transform.getOrigin().x();
        yinit_ = transform.getOrigin().y();

        mCurrentMap_.setInitPisition(xinit_, yinit_);

        igm_.vis_.reset(new hmpl::ExplorationTransformVis("PT_map"));

        as_.registerPreemptCallback(boost::bind(&ExploreAction::preemptCB, this));

        as_.start();

    }

    ~ExploreAction(void) {
    }

#ifdef InfoGain
    void run(const autonomous_exploration::ExploreGoalConstPtr &goal)
    {
        Rate loop_rate(4);

        // zwk todo
//		double current_gain = mCurrentMap_.getGainConst();
        double current_gain = initial_gain_;

        int count = 0;
        double target_x, target_y;

        auto start = hmpl::now();
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


            int explore_target = exploreByInfoGain(&mCurrentMap_, pos_index);
//            int explore_target =  exploreByBfs(&mCurrentMap_, pos_index);
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

        auto end = hmpl::now();
        std::cout << "once explore cost time:" << hmpl::getDurationInSecs(start, end) << "\n";

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
#endif

    void run(const autonomous_exploration::ExploreGoalConstPtr &goal) {
        Rate loop_rate(4);


        int count = 0;
        std::vector<geometry_msgs::Point> frontier_array;
        std::vector<PoseWrap> final_goals;
        auto start = hmpl::now();
        while (ok() && as_.isActive()) {
            loop_rate.sleep();

            unsigned int pos_index;

            if (!getMap()) {
                ROS_ERROR("Could not get a map");
                as_.setPreempted();
                return;
            }

            if (!mCurrentMap_.getCurrentPosition(pos_index)) {
                ROS_ERROR("Could not get a position");
                as_.setPreempted();
                return;
            }

            if (!getBinaryMap()) {
                ROS_ERROR("Could not get a Binary map");
                as_.setPreempted();
                return;
            }

//            binary_gom_pub_.publish(binary_ogm_);
            // Initialize gridmap with ogm (all is same)
            grid_map::GridMapRosConverter::fromOccupancyGrid(binary_ogm_, igm_.obs, igm_.maps);
            // value replacement
            grid_map::Matrix& grid_data = igm_.maps[igm_.obs];
            size_t size_x = igm_.maps.getSize()(0);
            size_t size_y = igm_.maps.getSize()(1);
            // pre-process
            // 0 : obstacle
            // 255 : free/unknown
            for (size_t idx_x = 0; idx_x < size_x; ++idx_x){
                for (size_t idx_y = 0; idx_y < size_y; ++idx_y) {
                    if (0.0 == grid_data(idx_x, idx_y)) {
                        grid_data(idx_x, idx_y) = igm_.FREE;
                    } else if(100.0 == grid_data(idx_x, idx_y)) {
                        grid_data(idx_x, idx_y) = igm_.OCCUPY;
                    }
                }
            }

            ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                     igm_.maps.getLength().x(), igm_.maps.getLength().y(),
                     igm_.maps.getSize()(0), igm_.maps.getSize()(1));
            auto start = hmpl::now();
            igm_.updateDistanceLayerCV();
            auto end = hmpl::now();
            std::cout << "fast DT update time [s]:" << hmpl::getDurationInSecs(start, end) << std::endl;

            // find frontier points
            frontier_array = exploreByBfs(&mCurrentMap_, pos_index);

            // find gridmap index
            std::vector<grid_map::Index> goals;
            grid_map::Index goal_index;
            BOOST_FOREACH(geometry_msgs::Point point, frontier_array) {
                            grid_map::Position pose(point.x, point.y);
                            bool flag1 = igm_.maps.getIndex(pose, goal_index);
                            if(!flag1) {
                                ROS_WARN("frontier point is out of map!");
                                continue;
                            }
                            goals.push_back(goal_index);
                        }
            ROS_INFO("goals of PT nums : %d \n", goals.size());

            start = hmpl::now();
            igm_.updateExplorationTransform(goals, 5, 10, 1.0);
            end = hmpl::now();
            igm_.vis_->publishVisOnDemand(igm_.maps, igm_.explore_transform);
            ROS_INFO( "PT map cost time %f \n" , hmpl::getDurationInSecs(start, end));

            start = hmpl::now();
            hmpl::Pose2D revised_start_pose;
            bool flag = grid_map_path_planning::adjustStartPoseIfOccupied(igm_.maps, start_point_, revised_start_pose,
                                                                          igm_.obs, igm_.dis, igm_.explore_transform);
            if(!flag) {
                std::cout << "start point is out of map" << '\n';
                continue;
            }

            std::vector<geometry_msgs::PoseStamped> result_path;
            grid_map_path_planning::findPathExplorationTransform(igm_.maps, revised_start_pose,
                                                                 result_path,igm_.obs, igm_.dis, igm_.explore_transform);
            end = hmpl::now();
            std::cout << "PT path cost time:" << hmpl::getDurationInSecs(start, end) << "\n";
            if(!result_path.empty()) {
                nav_msgs::Path path_msg;
                geometry_msgs::PoseStamped pose;
                for (auto &point_itr : result_path) {
                    pose = point_itr;
                    path_msg.header.frame_id = igm_.maps.getFrameId();
                    path_msg.header.stamp = ros::Time::now();
                    pose.header = path_msg.header;
                    path_msg.poses.push_back(pose);
                }
                path_publisher_.publish(path_msg);
            }


            if (frontier_array.size() > 0) {
//                final_goals = getCheapest(frontier_centroids, pos_index);
                break;
            } else {
                ROS_INFO("Got NO frontier_centroids!");
                // todo try something
            }
        }

        auto end = hmpl::now();
        std::cout << " explore cost time:" << hmpl::getDurationInSecs(start, end) << "\n";

        if (as_.isActive()) {
            if (final_goals.size() > 0) {
                as_.setSucceeded();
                ROS_INFO("Exploration was finished");
            } else {
                as_.setAborted();
                ROS_INFO("Exploration was interrupted");
            }
        } else {
            as_.setAborted();
            ROS_INFO("Exploration was aborted");
        }
    }


    void preemptCB() {
        ROS_INFO("Server received a cancel request.");
//		mCurrentMap_.generateMap();
        goalReached_ = -1;
//		ac_.cancelGoal();
        as_.setPreempted();
    }

    void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result) {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Robot reached the explore target");
            goalReached_ = 1;
        } else {
            goalReached_ = -1;
        }
    }

    void startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
        start_point_.position.x = start->pose.pose.position.x;
        start_point_.position.y = start->pose.pose.position.y;
        start_point_.orientation = tf::getYaw(start->pose.pose.orientation);
        std::cout << "get initial state." << std::endl;
    }

private:

    int exploreByInfoGain(GridMap *map, unsigned int start) {
        ROS_INFO("Starting exploration");

        map->clearArea(start);
        ROS_INFO("Cleared the area");

        unsigned int mapSize = map->getSize();
        // represent visit flag table
        double *plan = new double[mapSize];
        for (unsigned int i = 0; i < mapSize; i++) {
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

        std::vector<unsigned int> frontier_array, search_array;

        while (!queue.empty() /*&& !foundFrontier*/ && goalReached_ != 2) {
            next = queue.begin();
            distance = next->first;
            unsigned int index = next->second;
            queue.erase(next);

            ROS_DEBUG("Target distance: %f", distance);

            if (distance > minDistance_ && map->isFrontier(index)) {
                foundFrontier = index;
                publishMarker(index, 2);
                frontier_array.push_back(index);
            } else {
                unsigned int ind[4];

                ind[0] = index - 1;
                ind[1] = index + 1;
                ind[2] = index - map->getWidth();
                ind[3] = index + map->getWidth();

                for (unsigned int it = 0; it < 4; ++it) {
                    unsigned int i = ind[it];
                    if (map->isFree(i) && plan[i] == -1)  // free and unvisited, then insert
                    {
                        queue.insert(Entry(distance + linear, i));
                        plan[i] = distance + linear;

                        publishMarker(index, 1);
                        search_array.push_back(index);
                    }
                }
            }
        }

        delete[] plan;
        queue.clear();


        if (foundFrontier) {
//			publishMarkerArray(frontier_array, 2);
            return foundFrontier;
        } else {
//			publishMarkerArray(search_array, 1);
            return -1;
        }
    }

    std::vector<geometry_msgs::Point> exploreByBfs(GridMap *map, unsigned int start) {
        ROS_INFO("Starting exploration");

//        map->clearArea(start);
//        ROS_INFO("Cleared the area");
        FrontierSearch bfs_searcher(map->getMap());

        hmpl::Pose2D current_pos;
        geometry_msgs::Pose start_pose = mCurrentMap_.getCurrentLocalPosition();
        current_pos.position.x = start_pose.position.x;
        current_pos.position.y = start_pose.position.y;
        current_pos.orientation = util::modifyTheta(tf::getYaw(start_pose.orientation));
        std::list<Frontier> frontier_list = bfs_searcher.searchFrom(start, current_pos);
        ROS_INFO("frontier_list size: %d", frontier_list.size());

        //create placeholder for selected frontier
        Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        std::vector<unsigned int> frontier_array_centroid;
        std::vector<geometry_msgs::Point> frontier_array;
        std::vector<int> type_array;
        int type = 0, max = 0;
        float originX = map->getOriginX();
        float originY = map->getOriginY();
        float resolution = map->getResolution();
        BOOST_FOREACH(Frontier frontier, frontier_list) {

                        float x = frontier.centroid.x;
                        float y = frontier.centroid.y;
                        unsigned int index;

                        unsigned int X = (x - originX) / resolution;
                        unsigned int Y = (y - originY) / resolution;
                        map->getIndex(X, Y, index);
                        {
//                            frontier_array_centroid.push_back(index);  // insert centroid
//                            type_array.push_back(type + 1);
                            // show all frontiers
#ifdef DEBUG
                            BOOST_FOREACH(geometry_msgs::Point point, frontier.point_array) {

                                            frontier_array.push_back(point);
                                            float x = point.x;
                                            float y = point.y;
                                            unsigned int index;

                                            unsigned int X = (x - originX) / resolution;
                                            unsigned int Y = (y - originY) / resolution;

                                            map->getIndex(X, Y, index);
                                            frontier_array_centroid.push_back(index);
                                            type_array.push_back(type);
                                        }
#endif
                        }
                        type++;
                    }
#ifdef DEBUG
        publishMarkerArray(frontier_array_centroid, type_array);
#endif
        return frontier_array;
    }

    bool moveTo(unsigned int goal_index) {
        // add by zwk todo
        ROS_INFO("Failed to reach the goal");
        return false;

        Rate loop_rate(8);

        while (!ac_.waitForServer()) {
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
        ac_.sendGoal(move_goal, boost::bind(&ExploreAction::doneCb, this, _1, _2));

        goalReached_ = 0;

        while (goalReached_ == 0 && ok()) {
            loop_rate.sleep();
        }

        if (goalReached_ == 1) {
            ROS_INFO("Reached the goal");
            return true;
        }

        ROS_INFO("Failed to reach the goal");
        return false;
    }

    bool getMap() {
        if (!mGetMapClient_.isValid()) {
            return false;
        }

        nav_msgs::GetMap srv;

        if (!mGetMapClient_.call(srv)) {
            ROS_INFO("Could not get a map.");
            return false;
        }

        mCurrentMap_.update(srv.response.map);
//        ROS_INFO("Got new map of size %d x %d", mCurrentMap_.getWidth(), mCurrentMap_.getHeight());
//        ROS_INFO("Map resolution is: %f", mCurrentMap_.getResolution());

        return true;
    }

    bool getBinaryMap() {
        if (!mGetBinaryMapClient_.isValid()) {
            return false;
        }

        nav_msgs::GetMap srv;

        if (!mGetBinaryMapClient_.call(srv)) {
            ROS_INFO("Could not get a Binary map.");
            return false;
        }

        binary_ogm_ = srv.response.map;
//        unsigned int  mMapWidth = binary_ogm_.info.width;
//        unsigned int mMapHeight = binary_ogm_.info.height;
//        ROS_INFO("Got new binary map of size %d x %d", mMapWidth, mMapHeight);

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
        if (type == 1) {
            Color color(0, 0, 0.7);
            marker.setParams("free", pose.getPose(), 0.35, color.getColor());
        } else if (type == 2) {
            Color color(0, 0.7, 0);
            marker.setParams("frontier", pose.getPose(), 0.75, color.getColor());
        }

        mMarkerPub_.publish(marker.getMarker());
    }

    void publishMarkerArray(const std::vector<unsigned int> &index_array, std::vector<int> type) {
        visualization_msgs::MarkerArray markersMsg;
        for (int i = 0; i < index_array.size(); i++) {
            double x, y;
            mCurrentMap_.getOdomCoordinates(x, y, index_array[i]);
            PoseWrap pose(x, y);

            VisMarker marker;
            /*
              type:
                1: free area
                2: frontier
            */
            type[i] = type[i] % 4;
            if (type[i] == 0) {
                Color color(0, 0, 0.7);
                marker.setParams("frontier", pose.getPose(), 0.35, color.getColor(), i);
            } else if (type[i] == 1) {
                Color color(0, 0.7, 0);
                marker.setParams("frontier", pose.getPose(), 0.35, color.getColor(), i);
            } else if (type[i] == 2) {
                Color color(0.7, 0, 0);
                marker.setParams("frontier", pose.getPose(), 0.35, color.getColor(), i);
            } else if (type[i] == 3) {
                Color color(0, 1.0, 0);
                marker.setParams("frontier", pose.getPose(), 0.35, color.getColor(), i);
            } else if (type[i] == -1) {
                Color color(0, 1.0, 0);
                marker.setParams("frontier", pose.getPose(), 0.15, color.getColor(), i, 0.6);
            } else if (type[i] == -2) {
                Color color(0, 1.0, 0);
                marker.setParams("frontier", pose.getPose(), 0.75, color.getColor(), i);
            } else {
                ROS_WARN("TYPE INDEX ERROR!");
            }

            markersMsg.markers.push_back(marker.getMarker());
        }
        mMarkerPub_.publish(markersMsg);
    }

    std::vector<PoseWrap> getCheapest(std::vector<unsigned int> &frontier_centroids, unsigned int current_pos) {
        std::vector<PoseWrap> goals;
        goals.reserve(frontier_centroids.size());

        int too_close_counter = 0;
        int no_new_cell_counter = 0;
        int touch_obstacle = 0;
        int outside_of_the_map = 0;

        double max_robot_goal_dist = 0;
        double max_explored_cells = 0;
        std::list<ExplorationPoint> expl_point_list;
        unsigned int map_width = mCurrentMap_.getWidth();
        unsigned int map_height = mCurrentMap_.getHeight();
        double origin_x = mCurrentMap_.getOriginX();
        double origin_y = mCurrentMap_.getOriginY();
        // calculate angle_diff cost
        BOOST_FOREACH(unsigned int index, frontier_centroids) {
                        if(index > map_width * map_height) {
                            outside_of_the_map++;
                            ROS_WARN("Goal point is out of map!");
                            continue;
                        }
                        double mxs, mys;
                        geometry_msgs::Pose start_pose = mCurrentMap_.getCurrentLocalPosition();
                        mxs = start_pose.position.x;
                        mys = start_pose.position.y;

                        double gx, gy;
                        mCurrentMap_.getOdomCoordinates(gx, gy, index);

                        double distance2goal = util::calcDistance(mxs, mys, gx, gy);
                        if(distance2goal > max_robot_goal_dist) {
                            max_robot_goal_dist = distance2goal;
                        }
                        // Goal poses which are too close to the robot are discarded.
                        if(distance2goal < config_.min_goal_distance) {
                            too_close_counter++;
                            continue;
                        }

                        // Vehicle body collision checking
                        if(!mCurrentMap_.isFree(index)) {
                            touch_obstacle++;
                            continue;
                        }

                        // Ignore ecploration point if it is no real exploration point.
                        double numberOfExploredCells = mCurrentMap_.uFunction(index);
                        if (numberOfExploredCells > max_explored_cells) {
                            max_explored_cells = numberOfExploredCells;
                        }
                        if (numberOfExploredCells <= 5.0) {
                            no_new_cell_counter++;
                            continue;
                        }

                        // Calculate angular distance, will be [0,PI)
                        double goal_proj_x = gx-mxs;
                        double goal_proj_y = gy-mys;
                        double start_angle = util::modifyTheta(tf::getYaw(start_pose.orientation));
                        double goal_angle = util::modifyTheta(std::atan2(goal_proj_y,goal_proj_x));
                        PoseWrap goal_pose(gx, gy, goal_angle);
                        double angle_distance = util::calcDiffOfRadian(start_angle, goal_angle);

                        // Create exploration point and add it to a list to be sorted as soon
                        // as max_robot_goal_dist and max_explored_cells are known.
                        ExplorationPoint expl_point(goal_pose, numberOfExploredCells, angle_distance, distance2goal); // See documentation in ExplorationPoint.edgeFound.
                        expl_point_list.push_back(expl_point);
                    }
        ROS_INFO("%d of %d exploration points are uses: %d touches an obstacle, %d are too close to the robot, %d leads to no new cells, %d lies outside of the map",
                 expl_point_list.size(), frontier_centroids.size(), touch_obstacle, too_close_counter, no_new_cell_counter,
                 outside_of_the_map);
        std::list<ExplorationPoint>::iterator it = expl_point_list.begin();
        double max_value = 0;
        double min_value = std::numeric_limits<double>::max();
        double value = 0;
        for (; it != expl_point_list.end(); it++) {
            value = it->calculateOverallValue(config_.weights, max_explored_cells, max_robot_goal_dist);
            if (value > max_value)
                max_value = value;
            if (value < min_value)
                min_value = value;
        }
        // Higher values are better, so the optimal goal is the last one.
        expl_point_list.sort();
        PoseWrap expl_rbs;
        std::vector<unsigned int> goal_list;
        std::vector<int> type_array;
        int type = 0;
        ROS_INFO("Exploration point list:\n");
        for (auto rit = expl_point_list.crbegin(); rit != expl_point_list.crend(); ++rit) {
            expl_rbs = rit->explPose;
            goals.push_back(expl_rbs);
            ROS_INFO(
                    "Point(%4.2f, %4.2f, %4.2f) values: overall(%4.2f), expl(%4.2f), ang(%4.2f), dist(%4.2f)\n",
                    rit->explPose.x, rit->explPose.y, rit->explPose.theta, rit->overallValue,
                    rit->expl_value, rit->ang_value, rit->dist_value);
            unsigned int x = (rit->explPose.x - origin_x) / mCurrentMap_.getResolution();
            unsigned int y = (rit->explPose.y - origin_y) / mCurrentMap_.getResolution();
            unsigned int index;
            mCurrentMap_.getIndex(x, y, index);
            goal_list.push_back(index);
            type_array.push_back(type);
            type++;
        }


        if (!goals.empty()) {
            type_array[0] = -2;
            publishMarkerArray(goal_list, type_array);
        } else {
            ROS_INFO( "did not find any target, propably stuck in an obstacle.");
        }

        return goals;
    }


protected:
    NodeHandle nh_;
    actionlib::SimpleActionServer<autonomous_exploration::ExploreAction> as_;
    autonomous_exploration::ExploreFeedback feedback_;
    autonomous_exploration::ExploreResult result_;
    MoveBaseClient ac_;

    ServiceClient mGetMapClient_;
    GridMap mCurrentMap_;
    double minDistance_;
    double minGain_;
    double initial_gain_;
    double gainChangeFactor_;
    int goalReached_;

    Publisher mMarkerPub_;
    Publisher binary_gom_pub_;
    Publisher path_publisher_;
    Subscriber start_sub_;
    hmpl::InternalGridMap igm_;
    ServiceClient mGetBinaryMapClient_;
    nav_msgs::OccupancyGrid binary_ogm_;

    hmpl::Pose2D start_point_;




public:
    config::Config config_;
};

int main(int argc, char **argv) {
    init(argc, argv, "explore_server_node");
    config::Config cfg;

    cfg.min_goal_distance = 3.0;
    cfg.weights = config::Weights(1, 1, 1);
    cfg.vehicle_length = 4.9;
    cfg.vehicle_width = 2.8;
    cfg.base2back = 1.09;

    ExploreAction explore("explore", cfg);

    spin();

    return 0;
}
