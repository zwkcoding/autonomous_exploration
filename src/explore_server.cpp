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
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <opt_utils/opt_utils.hpp>
#include <internal_grid_map/internal_grid_map.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <path_transform/path_planning.hpp>
#include <car_model/car_geometry.hpp>
#include <iv_explore_msgs/ExploreAction.h>
#include <iv_explore_msgs/PlannerControlAction.h>

#include "autonomous_exploration/bfs_frontier_search.hpp"
#include "autonomous_exploration/util.hpp"
#include "autonomous_exploration/Config.hpp"
#include <autonomous_exploration/GridMap.h>
#include <autonomous_exploration/VisMarker.h>
#include <autonomous_exploration/PoseWrap.h>
#include <autonomous_exploration/Color.h>

using namespace tf;
using namespace ros;
using namespace frontier_exploration;
using namespace cv;

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
                                                          explore_action_name_(name),astar_planner_action_name_("astar_move_base"),
                                                          ac_("astar_move_base", true), config_(cfg){

//		mMarkerPub_ = nh_.advertise<visualization_msgs::Marker>("vis_marker", 2);
        original_frontier_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("origin_frontier_marker", 1);
        filtered_frontier_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("filtered_frontier_marker", 1);
        sparsed_frontier_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("sparsed_frontier_marker", 1);
        sorted_frontier_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("sorted_frontier_marker", 1);
        final_frontier_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("final_frontier_marker", 1);
        mGetMapClient_ = nh_.serviceClient<nav_msgs::GetMap>(std::string("global_map"));
        mGetCoverMapClient_ = nh_.serviceClient<nav_msgs::GetMap>(std::string("cover_map"));

//        start_sub_ = nh_.subscribe("/initialpose", 1, &ExploreAction::startCb, this);
        path_publisher_ = nh_.advertise<nav_msgs::Path>("PT_path", 1, true);

        ros::NodeHandlePtr pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));
        double laser_range = 8.0;
//		pnode_->param("laser_max_range", laser_range, 8.);
        mCurrentMap_.setLaserRange(laser_range);

        int lethal_cost;
        pnode_->param("obs_threshold", lethal_cost, 80);
        mCurrentMap_.setLethalCost(lethal_cost);

        pnode_->param("gain_threshold", initial_gain_, 20.);
        mCurrentMap_.setGainConst(initial_gain_);

        double robot_radius = config_.vehicle_length / 2;
//		pnode_->param("robot_radius", robot_radius, 2.);
        mCurrentMap_.setRobotRadius(robot_radius);

        std::string map_path;
        std::string temp = "/";
        pnode_->param("map_path", map_path, temp);
        mCurrentMap_.setPath(map_path);

        minDistance_ = 5.0;
//		pnode_->param("min_distance", minDistance_, 5.);
        pnode_->param("min_gain_threshold", minGain_, 0.5);
        pnode_->param("roi_size_gain_change", gainChangeFactor_, 1.2);

        start_point_ = hmpl::Pose2D(10, 10, M_PI);

        // keep initial pose
        std::string local_map_frame_name_, global_map_frame_name_;
        pnode_->param<std::string>("local_map_frame_name", local_map_frame_name_, "base_link");
        pnode_->param<std::string>("global_map_frame_name", global_map_frame_name_, "/odom");
        pnode_->param<double>("sparse_distance_between_frontiers", sparse_dis_, 1);
        pnode_->param<double>("expect_info_radius", expect_info_radius_, 2);
        pnode_->param<int>("min_unknown_cells_", min_unknown_cells_, 50);
        pnode_->param<double>("initial_roi_radius_", initial_roi_radius_, 25);
        pnode_->param<double>("max_roi_radius_", max_roi_radius_, 50);
        pnode_->param<double>("explored_rate_thresh", explored_rate_thresh_, 80);



        double xinit_ = 0, yinit_ = 0;
        tf::TransformListener mTfListener;
        tf::StampedTransform transform;
        int temp0 = 0;
        // wait here until receive tf tree
        while (temp0 == 0) {
            try {
                temp0 = 1;
                mTfListener.lookupTransform(global_map_frame_name_, local_map_frame_name_, ros::Time(0), transform);
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

        // initial car geometry parameters
        InitCarGeometry(car_);

        // register Preempt callbacks
        as_.registerPreemptCallback(boost::bind(&ExploreAction::preemptCB, this));
        as_.start();

    }

    ~ExploreAction(void) {
    }

#ifdef InfoGain
    void run(const autonomous_exploration::ExploreGoalConstPtr &goal)
    {
        Rate loop_rate(4);

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

    void run(const iv_explore_msgs::ExploreGoalConstPtr &goal) {

        Rate loop_rate(4);
        int count = 0;
        // final results sent tp planning module
        std::vector<PoseWrap> final_goals;
        // set scalable ROI size for bfs extracting frontier routine
        double roi_radius = initial_roi_radius_;
        static bool use_cover_map = false;
        auto start = hmpl::now();
        while (ok() && as_.isActive() && roi_radius <= max_roi_radius_ ) {
            loop_rate.sleep();

            std::vector<geometry_msgs::Point> frontier_array;
            std::vector<geometry_msgs::Point> filtered_frontier_array;
            std::vector<geometry_msgs::Point> sparsed_frontier_array;

            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_WARN("Explore Server received a cancel request.");
                goalReached_ = -1;
                ac_.cancelGoal();
                ROS_INFO("%s: Preempted", explore_action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                return;
            }

            if(!use_cover_map) {
                if (!getMap()) {
                    ROS_ERROR("Could not get a Global map");
                    as_.setPreempted();
                    return;
                }
            } else {
                if (!getCoverMap()) {
                    ROS_ERROR("Could not get a Cover map");
                    as_.setPreempted();
                    return;
                }

            }

            unsigned int pos_index;
            if (!mCurrentMap_.getCurrentPosition(pos_index)) {
                as_.setPreempted();
                // todo outside of area, go back
                return;
            }

            binary_ogm_ = mCurrentMap_.getMap();
            /*cv::Mat m1 = cv::Mat(binary_ogm_.info.height, binary_ogm_.info.width,
                                 CV_8UC1);
            cv::Mat mat_src = cv::Mat(binary_ogm_.info.height, binary_ogm_.info.width, CV_8SC1);
            //copy vector to mat
            memcpy(mat_src.data, binary_ogm_.data.data(), binary_ogm_.data.size() * sizeof(signed char));
            mat_src.setTo(cv::Scalar(0), mat_src == -1);

            mat_src.convertTo(m1, CV_8UC1);
            imshow("binary_test", m1);
            waitKey(1);
            binary_ogm_.data.assign((int8_t*)m1.datastart, (int8_t*)m1.dataend);*/

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
                    } else {
//                    grid_data(idx_x, idx_y) = igm_.OCCUPY;
                        // warn : view unknown as free
                        grid_data(idx_x, idx_y) = igm_.FREE;
                    }
                }
            }
            igm_.updateDistanceLayerCV();

            // update roi_radius for bfs frontiers extracting func
            bfs_searcher_.setPolygonRadius(roi_radius);
            // find frontier points(no filter)
            if(!use_cover_map) {
                frontier_array = exploreByBfs(&mCurrentMap_, pos_index);
            } else {
                frontier_array =  detectFrontiersByOpenCV(mCurrentMap_.getMap());
            }
            ROS_INFO("origin frontier size : %d", frontier_array.size());

            // filter frontier points(collision, isolated-frontiers--no info_gain)
            float originX = mCurrentMap_.getOriginX();
            float originY = mCurrentMap_.getOriginY();
            float resolution = mCurrentMap_.getResolution();
            nav_msgs::OccupancyGrid tmp = mCurrentMap_.getMap();
            cv::Mat m = cv::Mat(tmp.info.height, tmp.info.width,
                        CV_8SC1); // initialize matrix of signed char of 1-channel where you will store vec data
            cv::Mat m0 = cv::Mat(tmp.info.height, tmp.info.width,
                                 CV_8UC1);
            //copy vector to mat
            memcpy(m.data, tmp.data.data(), tmp.data.size() * sizeof(signed char));
//            imshow("m0", m);
            m.setTo(cv::Scalar(120), m == -1);
//            imshow("m1", m);
//            m0.setTo(cv::Scalar(120), m == -1);
//            m0.setTo(cv::Scalar(0), m == 0);
//            m0.setTo(cv::Scalar(100), m == 100);

            m.convertTo(m0, CV_8UC1);
//            imshow("m0", m0);
            cv::waitKey(1);

            cv::Mat roiMat = m0.clone();
//        std::cerr << roiMat.type() << " " << roiMat.channels() << " " << roiMat.size() << std::endl;

            cv::threshold( roiMat, roiMat, 110, 255, cv::THRESH_BINARY);
            int count_white = cv::countNonZero(roiMat); // white cell is unknown cell
            int rows = roiMat.rows;
            int cols = roiMat.cols;
            double explored_rate = 100.0 - count_white * 100.0 / (rows * cols);
            if(explored_rate > explored_rate_thresh_) {
                use_cover_map = true;
            }
            ROS_WARN_THROTTLE(3, "USE COVER MAP FLAG : %s", false == use_cover_map ? "FALSE":"TRUE");

            int info_radius = static_cast<int> (expect_info_radius_ / resolution);
            // todo filter initial frontier on vehicle body when vehicle start at first
            BOOST_FOREACH(geometry_msgs::Point point, frontier_array) {
                             // Define the robot as square
                            grid_map::Position pos(point.x, point.y);

                            float x = point.x;
                            float y = point.y;
                            unsigned int index;

                            unsigned int X = (x - originX) / resolution;
                            unsigned int Y = (y - originY) / resolution;
                            if (this->igm_.maps.isInside(pos)) {
                                double clearance = this->igm_.getObstacleDistance(pos);
                                if (clearance < config_.vehicle_length * 1.5 / 2) {
                                    // the big circle is not collision-free, then do an exact
                                    // collision checking
                                } else { // collision-free
                                    if( mCurrentMap_.getIndex(X, Y, index) ) {
                                         int unknown_cells = countUnknownCells(m0, info_radius, X, Y);
                                        // 40% occupancy
                                         if(unknown_cells > min_unknown_cells_) {
                                             // filter isolated frontiers(circle)
                                            filtered_frontier_array.push_back(point);
                                        }
                                    }
                                }
                            }
                        }
#ifdef DEBUG
            if(filtered_frontier_pub_.getNumSubscribers() > 0) {
                publishMarkerArray(filtered_frontier_pub_, "filtered_frontiers", filtered_frontier_array);
            }
#endif
            ROS_INFO("filtered frontier size : %d", filtered_frontier_array.size());

            if(filtered_frontier_array.empty()) {
                ROS_WARN("No frontiers after filtering, Enlarging search area ...");
                roi_radius *= gainChangeFactor_;
                ROS_INFO("ROI_radius was increased to: %f", roi_radius);
                continue;
            }
            // sparse frontier points
           sampleFrontierArea(filtered_frontier_array, sparsed_frontier_array);
#ifdef DEBUG
            if(sparsed_frontier_pub_.getNumSubscribers() > 0) {
                publishMarkerArray(sparsed_frontier_pub_, "sparsed_frontiers", sparsed_frontier_array);
            }
#endif
            ROS_INFO("sparsed frontier size : %d", sparsed_frontier_array.size());

            if (sparsed_frontier_array.size() > 0) {
                final_goals = getCheapest(m0, info_radius, sparsed_frontier_array, pos_index);
                // send goal to astar_action_client, awaiting for results
                moveTo(final_goals);
                break;
            } else {
                ROS_WARN("Got NO frontier after sparsing! Enlarging search area, BUT Sparsing operation is ERROR!!!");
                roi_radius *= gainChangeFactor_;
                ROS_INFO("ROI_radius was increased to: %f", roi_radius);
                continue;
            }


#ifdef PT
            // find gridmap index
            std::vector<grid_map::Index> goals;
            grid_map::Index goal_index;
            BOOST_FOREACH(geometry_msgs::Point point, sparsed_frontier_array) {
                            grid_map::Position pose(point.x, point.y);
                            bool flag1 = igm_.maps.getIndex(pose, goal_index);
                            if(!flag1) {
                                ROS_WARN("frontier point is out of map!");
                                continue;
                            }
                            goals.push_back(goal_index);
                        }
            ROS_INFO("goals of PT nums : %d \n", goals.size());

            auto start = hmpl::now();
            igm_.updateExplorationTransform(goals, 5, 10, 1.0);
            auto end = hmpl::now();
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
#endif

        }
        auto end = hmpl::now();
        std::cout << " explore cost time:" << hmpl::getDurationInSecs(start, end) << "\n";

        if (as_.isActive()) {
            if (final_goals.size() > 0 && 1 == goalReached_) {
                as_.setSucceeded();
                ROS_WARN("Exploration was finished");
            } else {
                as_.setAborted();
                ROS_WARN("Exploration was failed: maybe really no frontier(search area is max!) or goal is unaccessible!!!");
            }
        } else {
            ROS_WARN("Explore action server was prempted!!");
//            as_.setPreempted();
        }
    }


    // deprecated! could not enter when "run" func is running !
    void preemptCB() {
        ROS_INFO("Explore Server received a cancel request.");
//		mCurrentMap_.generateMap();
        goalReached_ = -1;
		ac_.cancelGoal();
        as_.setPreempted();
    }

    void doneCb(const actionlib::SimpleClientGoalState &state, const iv_explore_msgs::PlannerControlResultConstPtr &result) {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("vehicle reached the explore target");
            goalReached_ = 1;
        } else {
            goalReached_ = -1;
        }
    }

    void feedbackCb(const iv_explore_msgs::PlannerControlFeedbackConstPtr &feedback) {
        ROS_INFO("goal(%f, %f) is chosen!", feedback->current_goal.position.x, feedback->current_goal.position.y);
    }

    void startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) {
        start_point_.position.x = start->pose.pose.position.x;
        start_point_.position.y = start->pose.pose.position.y;
        start_point_.orientation = tf::getYaw(start->pose.pose.orientation);
        std::cout << "get initial state." << std::endl;
    }

private:


    void InitCarGeometry(hmpl::CarGeometry &car) {
        car.setBase2Back(config_.base2back);
        car.setVehicleLength(config_.vehicle_length);
        car.setVehicleWidth(config_.vehicle_width);
        car.setWheebase(config_.vehicle_wheelbase);
        car.buildCirclesFromFootprint();
    }

    bool isSingleStateCollisionFree(const hmpl::State &current) {
        // get the footprint circles based on current vehicle state in global frame
        std::vector<hmpl::Circle> footprint = this->car_.getCurrentCenters(current);
        // footprint checking
        for (auto &circle_itr : footprint) {
            grid_map::Position pos(circle_itr.position.x, circle_itr.position.y);
            // complete collision checking
            if (this->igm_.maps.isInside(pos)) {
                double clearance = this->igm_.getObstacleDistance(pos);
                if (clearance < circle_itr.r) {  // collision
                    // less than circle radius, collision
                    return false;
                }
            } else {
                // beyond boundaries , collision
                return false;
            }
        }
        // all checked, current state is collision-free
        return true;
    }

    bool isSingleStateCollisionFreeImproved(const hmpl::State &current) {
        // current state is in ogm: origin(0,0) is on center
        // get the bounding circle position in global frame
        hmpl::Circle bounding_circle = this->car_.getBoundingCircle(current);

        grid_map::Position pos(bounding_circle.position.x, bounding_circle.position.y);
        if (this->igm_.maps.isInside(pos)) {
            double clearance = this->igm_.getObstacleDistance(pos);
            if (clearance < bounding_circle.r) {
                // the big circle is not collision-free, then do an exact
                // collision checking
                return (this->isSingleStateCollisionFree(current));
            } else { // collision-free
                return true;
            }
        } else {  // beyond the map boundary
            return false;
        }
    }

    void sampleFrontierArea(std::vector<geometry_msgs::Point> &filtered_frontier_array,
                            std::vector<geometry_msgs::Point> &sparsed_frontier_array) {
        // not sparse when total frontier is little
        if (filtered_frontier_array.size() < 20) {
            ROS_INFO("filtered_frontiers numbers is little(less than 20), not sparse !!!");
            sparsed_frontier_array = filtered_frontier_array;
            return;
        }
        sparsed_frontier_array.reserve(filtered_frontier_array.size());
        sparsed_frontier_array.push_back(filtered_frontier_array[0]);

        size_t idx = 0;
        while (idx < filtered_frontier_array.size() - 2) {
            //std::cout << "idx: " << idx << " size: " << path_in.size() <<  "\n";
            const geometry_msgs::Point &current_index(filtered_frontier_array[idx]);

            for (size_t test_idx = idx + 2; test_idx < filtered_frontier_array.size(); ++test_idx) {
                const geometry_msgs::Point &test_index = filtered_frontier_array[test_idx];

                if (util::calcDistance(current_index.x, current_index.y, test_index.x, test_index.y) > sparse_dis_) {
                    idx = test_idx - 1;
                    break;
                } else {
                    if (test_idx == (filtered_frontier_array.size() - 1)) {
                        idx = test_idx;
                        break;
                    }
                }
            }

            sparsed_frontier_array.push_back(filtered_frontier_array[idx]);
        }

        if (idx < filtered_frontier_array.size()) {
            sparsed_frontier_array.push_back(filtered_frontier_array.back());
        }

        return ;
    }

    int countUnknownCells(const cv::Mat &mapData, int radius,  unsigned int x, unsigned int y) {
        cv::Rect rect1;
        int num_cells_radius = radius;
        // x,y is top left corner point
        rect1.x = x - num_cells_radius ;
        rect1.y = y - num_cells_radius;
        rect1.height = rect1.width = 2 * num_cells_radius;
        rect1 = rect1 & cv::Rect(0, 0, mapData.cols, mapData.rows);  // inside mat
        cv::Mat roiMat = mapData(rect1).clone();
//        std::cerr << roiMat.type() << " " << roiMat.channels() << " " << roiMat.size() << std::endl;

        cv::threshold( roiMat, roiMat, 110, 255, cv::THRESH_BINARY);
        int count_white = cv::countNonZero(roiMat); // white cell is unknown cell

        return count_white;
    }


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


    std::vector<PoseWrap> getCheapest(const cv::Mat &mapData, int radius, std::vector<geometry_msgs::Point> &frontier_centroids,
                                      unsigned int current_pos) {
        std::vector<PoseWrap> goals;
        goals.reserve(frontier_centroids.size());

        int too_close_counter = 0;
        int no_new_cell_counter = 0;
        int touch_obstacle = 0;
        int outside_of_the_map = 0;

        double max_robot_goal_dist = 0;
        int max_explored_cells = 0;
        std::list<ExplorationPoint> expl_point_list;
        unsigned int map_width = mCurrentMap_.getWidth();
        unsigned int map_height = mCurrentMap_.getHeight();
        double origin_x = mCurrentMap_.getOriginX();
        double origin_y = mCurrentMap_.getOriginY();
        double resolution = mCurrentMap_.getResolution();
        // calculate angle_diff cost
        BOOST_FOREACH(geometry_msgs::Point pt, frontier_centroids) {
                        double mxs, mys;
                        geometry_msgs::Pose start_pose = mCurrentMap_.getCurrentLocalPosition();
                        mxs = start_pose.position.x;
                        mys = start_pose.position.y;

                        double gx, gy;
                        gx = pt.x;
                        gy = pt.y;
                        double distance2goal = util::calcDistance(mxs, mys, gx, gy);
                        if(distance2goal > max_robot_goal_dist) {
                            max_robot_goal_dist = distance2goal;
                        }
//                        // Goal poses which are too close to the robot are discarded.
                        if(distance2goal < config_.min_goal_distance) {
                            too_close_counter++;
                            continue;
                        }

                        unsigned int X = (gx - origin_x) / resolution;
                        unsigned int Y = (gy - origin_y) / resolution;
                        // Ignore ecploration point if it is no real exploration point.
                        int numberOfExploredCells = countUnknownCells(mapData, radius, X, Y);
                        if (numberOfExploredCells > max_explored_cells) {
                            max_explored_cells = numberOfExploredCells;
                        }
                        if (numberOfExploredCells <= 50) {
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
                        ExplorationPoint expl_point(goal_pose, numberOfExploredCells, angle_distance, distance2goal);
                        expl_point_list.push_back(expl_point);
                    }
        ROS_INFO("%d of %d exploration points are uses:  %d are too close to the robot, %d leads to no new cells",
                 expl_point_list.size(), frontier_centroids.size(),  too_close_counter, no_new_cell_counter);
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
       // ROS_INFO("Exploration point list:\n");
        for (auto rit = expl_point_list.crbegin(); rit != expl_point_list.crend(); ++rit) {
            expl_rbs = rit->explPose;
            goals.push_back(expl_rbs);
        //    ROS_INFO(
        //            "Point(%4.2f, %4.2f, %4.2f) values: overall(%4.2f), expl(%4.2f), ang(%4.2f), dist(%4.2f)\n",
        //            rit->explPose.x, rit->explPose.y, rit->explPose.theta, rit->overallValue,
        //            rit->expl_value, rit->ang_value, rit->dist_value);
            unsigned int x = (rit->explPose.x - origin_x) / mCurrentMap_.getResolution();
            unsigned int y = (rit->explPose.y - origin_y) / mCurrentMap_.getResolution();
            unsigned int index;
            mCurrentMap_.getIndex(x, y, index);
            goal_list.push_back(index);
            type_array.push_back(type);
            type++;
        }

        if (!goals.empty()) {
            int max = goal_list.size() > 10 ? 10 : goal_list.size();
            for(int i = 0; i < max; i++) {
                type_array[i] = -2;
            }
        } else {
            ROS_WARN( "did not find any target, propably stuck in an obstacle.");
        }
#ifdef DEBUG
        if(sorted_frontier_pub_.getNumSubscribers() > 0) {
            publishMarkerArray(sorted_frontier_pub_, "sorted_frontiers", goal_list, type_array);
        }
#endif

        return goals;
    }

    std::vector<geometry_msgs::Point> detectFrontiersByOpenCV(const nav_msgs::OccupancyGrid &map) {
        std::vector<geometry_msgs::Point> frontiers;
        cv::Mat m1 = cv::Mat(map.info.height, map.info.width,
                             CV_8SC1); // initialize matrix of signed char of 1-channel where you will store vec data
        cv::Mat mat_src = cv::Mat(map.info.height, map.info.width, CV_8UC1);
        //copy vector to mat
        memcpy(m1.data, map.data.data(), map.data.size() * sizeof(signed char));
//        imshow("ooooo+", m1);

//        Mat m0;
//        m1.copyTo(m0);

        mat_src.setTo(cv::Scalar(205), m1 == -1);
        mat_src.setTo(cv::Scalar(0), m1 == 100);
        mat_src.setTo(cv::Scalar(255), m1 == 0);

//        mat_src.convertTo(mat_src, CV_8UC1);
//        imshow("delete_000000000bs+", mat_src);


        int thresh = 30;
        cv::Mat tmp, tmp_canny_output;
        inRange(mat_src, 0, 1, tmp);

        blur(mat_src, mat_src, Size(3, 3)); //滤波
        Canny(mat_src, tmp_canny_output, thresh, thresh * 3, 3);




//        GaussianBlur( mat_src, mat_src, Size(3,3), 0.1, 0, BORDER_DEFAULT );

        //定义Canny边缘检测图像
        Mat canny_output, canny_output_r, dst;
        std::vector<std::vector<Point2i>> contours;
        std::vector<Vec4i> hierarchy;
        //利用canny算法检测边缘
        Canny(tmp, canny_output, thresh, thresh * 3, 3);
//        namedWindow("canny", CV_WINDOW_AUTOSIZE);
//        flip(canny_output, canny_output_r, 0);
//        imshow("canny", canny_output_r);
//        moveWindow("canny", 550, 20);
        //查找轮廓
        findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point2i(0, 0));
        drawContours(tmp, contours, -1, 255, 5);
        bitwise_not(tmp, tmp);
        bitwise_and(tmp, tmp_canny_output, dst);
//        imshow("delete_obs", dst);
        findContours(dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point2i(0, 0));
        drawContours(dst, contours, -1, 255, 1);
//        imshow("frontiers", dst);

        //计算轮廓矩
        std::vector<Moments> mu(contours.size());
        for (int i = 0; i < contours.size(); i++) {
            mu[i] = moments(contours[i], false);
        }
        //计算轮廓的质心
        std::vector<Point2f> mc(contours.size());
        for (int i = 0; i < contours.size(); i++) {
            mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
        }
        //画轮廓及其质心并显示
        Mat drawing = Mat::zeros(dst.size(), CV_8UC3);
        RNG G_RNG(1234);

        for (int i = 0; i < contours.size(); i++) {
            Scalar color = Scalar(G_RNG.uniform(0, 255), G_RNG.uniform(0, 255), G_RNG.uniform(0, 255));
            drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point2i());
            circle(drawing, mc[i], 5, Scalar(0, 0, 255), -1, 8, 0);
//            rectangle(drawing, boundingRect(contours.at(i)), cvScalar(0, 255, 0));
            char tam[100];
            sprintf(tam, "(%0.0f,%0.0f)", mc[i].x, mc[i].y);
//            putText(drawing, tam, Point(mc[i].x, mc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(255, 0, 255), 1);
        }
        flip(drawing, drawing, 0);
//        namedWindow("Contours", CV_WINDOW_AUTOSIZE);
//        imshow("Contours", drawing);
//        moveWindow("Contours", 1100, 20);

        if (0) {
            //对每个轮廓的点集 找逼近多边形
            std::vector<std::vector<Point2i>> approxPoint(contours.size());
            for (int i = 0; i < (int) contours.size(); i++) {
                approxPolyDP(contours[i], approxPoint[i], 3, true);
            }

            /******************************************绘制曲线的方式********************************************/
            //用绘制轮廓的函数   绘制曲线
            Mat drawImage = Mat::zeros(dst.size(), CV_8UC3);
            for (int i = 0; i < (int) contours.size(); i++) {
                Scalar color = Scalar(G_RNG.uniform(0, 255), G_RNG.uniform(0, 255), G_RNG.uniform(0, 255));
                // 绘制凸包
                drawContours(drawImage, approxPoint, i, color, 1);

//                drawContours(drawImage, contours, i, Scalar(255, 255, 255), 1);
            }
//            flip(drawImage, drawImage, 0);

            imshow("lines", drawImage);
        } else {
            cv::Mat mat_canny, mat_canny_bgr;
            cv::RNG rng(0xFFFFFFFF);
//            Canny(dst, mat_canny, 50, 200);
//            imshow("delete_obs++", mat_canny);

            cvtColor(dst, mat_canny_bgr, CV_GRAY2BGR);
            std::vector<cv::Vec4f> lines;
            // Resolution: 1 px and 180/32 degree. last two para is important!
            HoughLinesP(dst, lines, 1, CV_PI / 32, 20, 20, 5);
            geometry_msgs::Point vec;
            for (size_t i = 0; i < lines.size(); i++) {
                cv::Vec4i l = lines[i];
                Scalar color = Scalar(G_RNG.uniform(0, 255), G_RNG.uniform(0, 255), G_RNG.uniform(0, 255));

                line(mat_canny_bgr, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 1, 8);
                Point2f pt;
                pt.x = (l[0] + l[2]) / 2;
                pt.y = (l[1] + l[3]) / 2;
                circle(mat_canny_bgr, pt, 5, Scalar(0, 0, 255), -1, 8, 0);

                vec.x = (l[0] + l[2]) / 2 * mCurrentMap_.getResolution() + mCurrentMap_.getOriginX();
                vec.y = (l[1] + l[3]) / 2 * mCurrentMap_.getResolution() + mCurrentMap_.getOriginY();
                vec.z = std::atan2(l[3] - l[1], l[2] - l[0]);
               /* {
                    int num_pixels_to_check = 10;
                    double expl_point_orientation = vec.theta;
                    int count_theta = countBlackPixels(mat_src, vec, num_pixels_to_check);
                    vec.theta = expl_point_orientation + M_PI;
                    int count_theta_pi = countBlackPixels(mat_src, vec, num_pixels_to_check);
                    if (abs(count_theta - count_theta_pi) >= 4) { // Difference is big enough.
                        if (count_theta_pi > count_theta) { // Use orientation with more black / unknown pixels.
                            expl_point_orientation = expl_point_orientation + M_PI;
                        }
                    }
                    vec.theta = expl_point_orientation;  //[0, 2pi]
                }*/

                frontiers.push_back(vec);
            }
            ROS_INFO("line number is %d", frontiers.size());

            flip(mat_canny_bgr, mat_canny_bgr, 0);
            namedWindow("lines", CV_WINDOW_AUTOSIZE);
            cv::imshow("lines", mat_canny_bgr);
//            moveWindow("lines", 550, 20);

        }

        cv::waitKey(1);
        return frontiers;
    }


        std::vector<geometry_msgs::Point> exploreByBfs(GridMap *map, unsigned int start) {
        ROS_INFO("Starting exploration");

//        map->clearArea(start);
//        ROS_INFO("Cleared the area");
        bfs_searcher_.getMap(map->getMap());

        hmpl::Pose2D current_pos;
        geometry_msgs::Pose start_pose = mCurrentMap_.getCurrentLocalPosition();
        current_pos.position.x = start_pose.position.x;
        current_pos.position.y = start_pose.position.y;
        current_pos.orientation = util::modifyTheta(tf::getYaw(start_pose.orientation));

        ROS_INFO_THROTTLE(5, "vehicle position in odom frame (%f[m], %f[m], %f[degree])",
                          current_pos.position.x, current_pos.position.y,
                          current_pos.orientation * 180 / M_PI);

        std::list<Frontier> frontier_list = bfs_searcher_.searchFrom(start, current_pos);
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
        if(original_frontier_pub_.getNumSubscribers() > 0) {
            publishMarkerArray(original_frontier_pub_, "original_frontiers", frontier_array_centroid, type_array);
        }
#endif
        return frontier_array;
    }

    bool moveTo(std::vector<PoseWrap> &goal_lists) {
        Rate loop_rate(8);
        while (!ac_.waitForServer()) {
            ROS_INFO("Waiting for the astar_move_base action server to come up");
            loop_rate.sleep();
        }

        iv_explore_msgs::PlannerControlGoal move_goal;

        move_goal.goals.header.frame_id = mCurrentMap_.getMap().header.frame_id;
        move_goal.goals.header.stamp = ros::Time::now();

        // todo at now, max chosen numbers is 10
        int max = goal_lists.size() > 10 ? 10 : goal_lists.size();
        geometry_msgs::Pose pose;
        std::vector<geometry_msgs::Point> final_frontier_array;
        geometry_msgs::Point point;

        if(0) {
            for (int i = 0; i < max; i++) {
                point.x = pose.position.x = goal_lists[i].x;
                point.y = pose.position.y = goal_lists[i].y;
                tf::quaternionTFToMsg(tf::createQuaternionFromYaw(goal_lists[i].theta), pose.orientation);
                final_frontier_array.push_back(point);
                move_goal.goals.poses.push_back(pose);
            }
        } else {
            if(goal_lists.size() < 5) {
                for (int i = 0; i < goal_lists.size(); i++) {
                    pose.position.x = goal_lists[i].x;
                    pose.position.y = goal_lists[i].y;
                    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(goal_lists[i].theta), pose.orientation);

                    move_goal.goals.poses.push_back(pose);
                }
            } else {
                size_t idx = 0;
                while (idx < goal_lists.size() - 2) {
                    //std::cout << "idx: " << idx << " size: " << path_in.size() <<  "\n";
                    const PoseWrap &current_index = goal_lists[idx];

                    for (size_t test_idx = idx + 2; test_idx < goal_lists.size(); ++test_idx) {
                        const PoseWrap &test_index = goal_lists[test_idx];

                        if (util::calcDistance(current_index.x, current_index.y, test_index.x, test_index.y) > 5) {
                            idx = test_idx - 1;
                            break;
                        } else {
                            if (test_idx == (goal_lists.size() - 1)) {
                                idx = test_idx;
                                break;
                            }
                        }
                    }

                    point.x = pose.position.x = goal_lists[idx].x;
                    point.y = pose.position.y = goal_lists[idx].y;
                    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(goal_lists[idx].theta), pose.orientation);

                    move_goal.goals.poses.push_back(pose);
                    final_frontier_array.push_back(point);
                    if(move_goal.goals.poses.size() > 9) {
                        break;
                    }

                }
            }

        }

#ifdef DEBUG

        if(final_frontier_pub_.getNumSubscribers() > 0) {
            publishMarkerArray(final_frontier_pub_, "final_frontiers", final_frontier_array);
        }
#endif


        ROS_WARN("Start sending goal to astar_action_server");
        ac_.sendGoal(move_goal, boost::bind(&ExploreAction::doneCb, this, _1, _2),
                     actionlib::SimpleActionClient<iv_explore_msgs::PlannerControlAction>::SimpleActiveCallback(),
                     boost::bind(&ExploreAction::feedbackCb, this, _1));

        goalReached_ = 0;

        while (0 == goalReached_  && ok()) {
            loop_rate.sleep();
            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_WARN("Explore Server received a cancel request, Cancel movement sent to PlanControl module!!");
                goalReached_ = -1;
                ac_.cancelGoal();
                ROS_INFO("%s: Preempted", explore_action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                break;
            }

        }

        if (goalReached_ == 1) {
            ROS_INFO("Vehicle has Reached the goal");
            return true;
        }

        ROS_INFO("Vehicle Failed to reach the goal maybe goal is cancelled or goal is really unaccessible!");
        return false;
    }

    bool getMap() {
        if (!mGetMapClient_.isValid()) {
            return false;
        }

        nav_msgs::GetMap srv;

        if (!mGetMapClient_.call(srv)) {
            ROS_INFO("Could not get a Global map.");
            return false;
        }

        mCurrentMap_.update(srv.response.map);
//        ROS_INFO("Got new map of size %d x %d", mCurrentMap_.getWidth(), mCurrentMap_.getHeight());
//        ROS_INFO("Map resolution is: %f", mCurrentMap_.getResolution());

        return true;
    }

    bool getCoverMap() {
        if (!mGetCoverMapClient_.isValid()) {
            return false;
        }

        nav_msgs::GetMap srv;

        if (!mGetCoverMapClient_.call(srv)) {
            ROS_INFO("Could not get a Cover map.");
            return false;
        }

        mCurrentMap_.update(srv.response.map);


        return true;
    }

    void publishMarker(unsigned int index, int type) {
        double x, y;
        mCurrentMap_.getOdomCoordinates(x, y, index);
        PoseWrap pose(x, y);
        std::string frame = mCurrentMap_.getMap().header.frame_id;

        VisMarker marker;
        /*
          type:
            1: free area
            2: frontier
        */
        if (type == 1) {
            Color color(0, 0, 0.7);
            marker.setParams(true, frame, "free", pose.getPose(), 0.35, color.getColor());
        } else if (type == 2) {
            Color color(0, 0.7, 0);
            marker.setParams(true, frame, "frontier", pose.getPose(), 0.75, color.getColor());
        }

        original_frontier_pub_.publish(marker.getMarker());
    }

    void publishMarkerArray(const ros::Publisher &pub, std::string marker_ns,
                            const std::vector<unsigned int> &index_array, std::vector<int> type) {
        static int last_point_nums = 0;
        visualization_msgs::MarkerArray markersMsg;

        for (int i = 0; i < index_array.size(); i++) {
            double x, y;
            mCurrentMap_.getOdomCoordinates(x, y, index_array[i]);
            PoseWrap pose(x, y);

            std::string frame = mCurrentMap_.getMap().header.frame_id;
            VisMarker marker;
            /*
              type:
                1: free area
                2: frontier
            */
            type[i] = type[i] % 4;
            if (type[i] == 0) {
                Color color(0, 0, 0.7);
                marker.setParams(true, frame, marker_ns, pose.getPose(), 0.35, color.getColor(), i);
            } else if (type[i] == 1) {
                Color color(0, 0.7, 0);
                marker.setParams(true, frame, marker_ns, pose.getPose(), 0.35, color.getColor(), i);
            } else if (type[i] == 2) {
                Color color(0.7, 0, 0);
                marker.setParams(true, frame, marker_ns, pose.getPose(), 0.35, color.getColor(), i);
            } else if (type[i] == 3) {
                Color color(0, 1.0, 0);
                marker.setParams(true, frame, marker_ns, pose.getPose(), 0.35, color.getColor(), i);
            } else if (type[i] == -1) {
                Color color(0, 1.0, 0);
                marker.setParams(true, frame, marker_ns, pose.getPose(), 0.15, color.getColor(), i, 0.6);
            } else if (type[i] == -2) {
                Color color(1.0, 0.0, 0);
                marker.setParams(true, frame, marker_ns, pose.getPose(), 0.75, color.getColor(), i);
            } else {
                ROS_WARN("TYPE INDEX ERROR!");
            }


            markersMsg.markers.push_back(marker.getMarker());
        }

        // delete old markers
        if(last_point_nums > index_array.size()) {
            for (int i = index_array.size(); i < last_point_nums; i++) {
                visualization_msgs::Marker marker;
                marker.ns = marker_ns;
                marker.id = i;
                marker.action = visualization_msgs::Marker::DELETE;
                markersMsg.markers.push_back(marker);
            }
        }


        pub.publish(markersMsg);
        last_point_nums = index_array.size();

    }

    void publishMarkerArray(const ros::Publisher &pub, std::string marker_ns,
                            const std::vector<geometry_msgs::Point> &points) {
        static int last_point_nums = 0;
        visualization_msgs::MarkerArray markersMsg;
        std::string frame = mCurrentMap_.getMap().header.frame_id;

        for (int i = 0; i < points.size(); i++) {
            PoseWrap pose(points[i].x, points[i].y);
            VisMarker marker;
            /*
              type:
                1: free area
                2: frontier
            */
            Color color(0, 0, 0.7);
            marker.setParams(true, frame, marker_ns, pose.getPose(), 0.35, color.getColor(), i);

            markersMsg.markers.push_back(marker.getMarker());
        }
        // delete old markers
        if(last_point_nums > points.size()) {
            for (int i = points.size(); i < last_point_nums; i++) {
                visualization_msgs::Marker marker;
                marker.ns = marker_ns;
                marker.id = i;
                marker.action = visualization_msgs::Marker::DELETE;
                markersMsg.markers.push_back(marker);
            }
        }

        pub.publish(markersMsg);

        last_point_nums = points.size();
    }


    void publishMarkerArray(const ros::Publisher &pub,
                            const std::vector<unsigned int> &index_array) {
        visualization_msgs::MarkerArray markersMsg;
        for (int i = 0; i < index_array.size(); i++) {
            double x, y;
            mCurrentMap_.getOdomCoordinates(x, y, index_array[i]);
            PoseWrap pose(x, y);

            std::string frame = mCurrentMap_.getMap().header.frame_id;
            VisMarker marker;
            /*
              type:
                1: free area
                2: frontier
            */
            Color color(0, 0, 0.7);
            marker.setParams(true, frame, "frontier", pose.getPose(), 0.35, color.getColor());

            markersMsg.markers.push_back(marker.getMarker());
        }
        pub.publish(markersMsg);
    }


protected:
    NodeHandle nh_;
    actionlib::SimpleActionServer<iv_explore_msgs::ExploreAction> as_;
    iv_explore_msgs::ExploreFeedback feedback_;
    iv_explore_msgs::ExploreResult result_;
    actionlib::SimpleActionClient<iv_explore_msgs::PlannerControlAction> ac_;

    ServiceClient mGetMapClient_;
    GridMap mCurrentMap_;
    double minDistance_;
    double minGain_;
    double initial_gain_;
    double gainChangeFactor_;
    int goalReached_;

    Publisher original_frontier_pub_;
    Publisher filtered_frontier_pub_;
    Publisher sparsed_frontier_pub_;
    Publisher sorted_frontier_pub_;
    Publisher final_frontier_pub_;

    Publisher binary_gom_pub_;
    Publisher path_publisher_;
    Subscriber start_sub_;
    hmpl::InternalGridMap igm_;
    ServiceClient mGetCoverMapClient_;
    nav_msgs::OccupancyGrid binary_ogm_;

    hmpl::Pose2D start_point_;

    // car model
    hmpl::CarGeometry car_;

    FrontierSearch bfs_searcher_;

    std::string explore_action_name_;
    std::string astar_planner_action_name_;

public:
    config::Config config_;

private:
    int min_unknown_cells_;
    double sparse_dis_;
    double expect_info_radius_;
    double initial_roi_radius_, max_roi_radius_;
    double explored_rate_thresh_;
};

int main(int argc, char **argv) {
    init(argc, argv, "explore_server_node");
    ros::NodeHandle nh("~");

    double vehicle_length, vehicle_width, vehicle_wheelbase, base2back;
    double info_coeff, angle_coeff, dis_coeff, min_frontier_distance;
    nh.param<double>("vehicle_length", vehicle_length, 4.9);
    nh.param<double>("vehicle_width", vehicle_width, 1.95);
    nh.param<double>("base2back", base2back, 1.09);
    nh.param<double>("vehicle_wheelbase", vehicle_wheelbase, 2.86);
    nh.param<double>("info_coeff", info_coeff, 1);
    nh.param<double>("angle_coeff", angle_coeff, 1);
    nh.param<double>("dis_coeff", dis_coeff, 1);
    nh.param<double>("min_frontier_distance", min_frontier_distance, 3.0);

    config::Config cfg;
    cfg.min_goal_distance = min_frontier_distance;
    cfg.weights = config::Weights(info_coeff, angle_coeff, dis_coeff);
    cfg.vehicle_length = vehicle_length;
    cfg.vehicle_width = vehicle_width;
    cfg.base2back = base2back;
    cfg.vehicle_wheelbase = vehicle_wheelbase;
    ExploreAction explore("explore", cfg);

    spin();

    return 0;
}
