//
// Created by kevin on 3/9/18.
//

#ifndef AUTONOMOUS_EXPLORATION_BFS_FRONTIER_SEARCH_HPP
#define AUTONOMOUS_EXPLORATION_BFS_FRONTIER_SEARCH_HPP

#include <vector>
#include <stdlib.h>
#include <queue>
#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include "nav_msgs/OccupancyGrid.h"
#include <boost/foreach.hpp>
#include "visualization_msgs/Marker.h"
#include "autonomous_exploration/GridMap.h"
#include "autonomous_exploration/util.hpp"

namespace frontier_exploration {
/**
 * @brief Thread-safe implementation of a frontier-search task for an input OGM.
 */


    struct Frontier {
    public:
        int size;
        float min_distance;
        geometry_msgs::Point initial;
        geometry_msgs::Point centroid;
        geometry_msgs::Point middle;
        std::vector<geometry_msgs::Point> point_array;
    };

    class FrontierSearch {

    public:

        bool nearestCell(unsigned int &result, unsigned int start, int val);

        std::vector<unsigned int> nhood4(unsigned int idx, unsigned int width, unsigned int height);

        std::vector<unsigned int> nhood8(unsigned int idx, unsigned int width, unsigned int height);

        void indexToReal(const nav_msgs::OccupancyGrid &map, const size_t index, float &x, float &y);

        /**
         * @brief Constructor for search task
         * @param mapData Reference to OccupancyGrid data to search.
         */
        FrontierSearch();

        void getMap(const nav_msgs::OccupancyGrid &mapData);

        /**
         * @brief Runs search implementation, outward from the start position
         * @param position Initial position to search from
         * @return List of frontiers, if any
         */
        std::list<Frontier> searchFrom(unsigned int pos, hmpl::Pose2D &current_pos);

        void setPolygonWidth(double r) { polygon_width_ = r; }

        void setPolygonLength(double r) { polygon_length_ = r; }


    protected:

        /**
         * @brief Starting from an initial cell, build a frontier from **valid(new and frontier)** **adjacent** cells
         * @param initial_cell Index of cell to start frontier building
         * @param reference Reference index to calculate position from
         * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
         * @return
         */
        Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool> &frontier_flag);

        /**
         * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate for a new frontier.
         * @param idx Index of candidate cell
         * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
         * @return
         */
        bool isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag);

        void addPolygon(double length, double width);

        void updatePolygon(hmpl::Pose2D &current_pos);

    private:

        nav_msgs::OccupancyGrid map_;
        unsigned int size_x_, size_y_;
        float resolution_, Xstarty_, Xstartx_;
        float search_radius, min_search_dis;

        std::vector<hmpl::Vector2D<double> >  base_interest_polygon_area_;
        std::vector<hmpl::Vector2D<double> >  current_interest_polygon_area_;

        double polygon_length_;
        double polygon_width_;
        double min_frontiers_nums_;

        ros::NodeHandle nh;
        ros::Publisher lines_pub;
        visualization_msgs::Marker points, line;
    };

}

#endif //AUTONOMOUS_EXPLORATION_BFS_FRONTIER_SEARCH_HPP
