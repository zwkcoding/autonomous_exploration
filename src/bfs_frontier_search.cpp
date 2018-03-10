//
// Created by kevin on 3/9/18.
//

#include "autonomous_exploration/bfs_frontier_search.hpp"

namespace frontier_exploration {

    FrontierSearch::FrontierSearch(const nav_msgs::OccupancyGrid &mapData)
            : map_(mapData),
              map_data_(mapData.data) {
        resolution_ = mapData.info.resolution;
        Xstartx_ = mapData.info.origin.position.x;
        Xstarty_ = mapData.info.origin.position.y;
        size_x_ = mapData.info.width;
        size_y_ = mapData.info.height;
    }

    std::list<Frontier> FrontierSearch::searchFrom(unsigned int pos) {

        std::list<Frontier> frontier_list;

        //initialize flag arrays to keep track of visited and frontier cells
        std::vector<bool> frontier_flag(size_x_ * size_y_, false);
        std::vector<bool> visited_flag(size_x_ * size_y_, false);

        //initialize breadth first search
        std::queue<unsigned int> bfs;

        //find closest clear cell to start search
        unsigned int nearest_clear_point;
        float ref_x, ref_y;
//        pos = (floor((position[1] - Xstarty_) / resolution_) * size_x_) +
//              (floor((position[0] - Xstartx_) / resolution_));
        if (pos > size_x_ * size_y_ - 1) {
            ROS_WARN("frontier search start point is out of map");
            return frontier_list;
        }
        if (nearestCell(nearest_clear_point, pos, FREE_SPACE)) {
            bfs.push(nearest_clear_point);
            indexToReal(map_, nearest_clear_point, ref_x, ref_y);
        } else {
            bfs.push(pos);
            indexToReal(map_, pos, ref_x, ref_y);

            ROS_WARN("Could not find nearby clear cell to start search");
        }
        visited_flag[bfs.front()] = true;


        float extend_x, extend_y;
        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();
            //iterate over 4-connected neighbourhood
            BOOST_FOREACH(unsigned int nbr, nhood4(idx, size_x_, size_y_)) {
                            //add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
                            if (map_data_[nbr] == FREE_SPACE && !visited_flag[nbr]) {
                                visited_flag[nbr] = true;
                                indexToReal(map_, nbr, extend_x, extend_y);
                                float dist = pow((pow((extend_x - ref_x), 2) + pow((extend_y - ref_y), 2)), 0.5);
                                // not to flood too far scope, for fast speed
                                // todo use local_map size
                                if (dist < 25) {
                                    bfs.push(nbr);
                                }
                                //check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
                            } else if (isNewFrontierCell(nbr, frontier_flag)) {
                                frontier_flag[nbr] = true;
                                Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                                // todo consider vehicle width pass ability
                                if (new_frontier.size > 30) {
                                    frontier_list.push_back(new_frontier);
                                }
                            }
                        }
        }

        return frontier_list;

    }

    Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                                              std::vector<bool> &frontier_flag) {

        //initialize frontier structure
        Frontier output;
        output.centroid.x = 0;
        output.centroid.y = 0;
        output.size = 1;
        output.min_distance = std::numeric_limits<double>::infinity();


        //push initial gridcell onto queue
        std::queue<unsigned int> bfs;
        bfs.push(initial_cell);
        float ref_x, ref_y;
        geometry_msgs::Point point;
        indexToReal(map_, initial_cell, ref_x, ref_x);
        point.x = ref_x;
        point.y = ref_y;
        output.point_array.push_back(point);

        indexToReal(map_, reference, ref_x, ref_y);
        output.initial.x = ref_x;
        output.initial.y = ref_y;

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            //try adding cells in 8-connected neighborhood to frontier
            BOOST_FOREACH(unsigned int nbr, nhood8(idx, size_x_, size_y_)) {
                            //check if neighbour is a potential frontier cell
                            if (isNewFrontierCell(nbr, frontier_flag)) {

                                //mark cell as frontier
                                frontier_flag[nbr] = true;
                                float wx, wy;
                                indexToReal(map_, nbr, wx, wy);

                                point.x = wx;
                                point.y = wy;
                                output.point_array.push_back(point);

                                //update frontier size
                                output.size++;

                                //update centroid of frontier
                                output.centroid.x += wx;
                                output.centroid.y += wy;

                                //determine frontier's distance from robot, going by closest gridcell to robot
                                double distance = sqrt(pow((double(ref_x) - double(wx)), 2.0) +
                                                       pow((double(ref_y) - double(wy)), 2.0));
                                // not to consider too far scope, for fast speed
                                if (distance < 25) {
                                    if (distance < output.min_distance) {
                                        output.min_distance = distance;
                                    }
                                    //add to queue for breadth first search
                                    bfs.push(nbr);
                                }

                            }
                        }
        }

        int middle_index = output.size / 2;
        output.middle.x = output.point_array[middle_index].x;
        output.middle.y = output.point_array[middle_index].y;

        //average out frontier centroid
        output.centroid.x /= output.size;
        output.centroid.y /= output.size;
        return output;
    }

    bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag) {

        // faster method
        if(0) {
            //check that cell is unknown and not already marked as frontier
            if (map_data_[idx] != NO_INFORMATION || frontier_flag[idx]) {
                return false;
            }
            BOOST_FOREACH(unsigned int nbr, nhood4(idx, size_x_, size_y_)) {
                            if (map_data_[nbr] == FREE_SPACE) {
                                return true;
                            }
                        }
            return false;
        }

        //check that cell is unknown and not already marked as frontier
        if (map_data_[idx] != FREE_SPACE || frontier_flag[idx]) {
            return false;
        }

        // todo
        //frontier cells should have at least one cell in 4-connected neighbourhood that is free
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, size_x_, size_y_)) {
                        if (map_data_[nbr] == NO_INFORMATION ) {
                            int no_inf_count = 0;
                            BOOST_FOREACH(unsigned int nbrr, nhood8(nbr, size_x_, size_y_))  {
                                if(map_data_[nbrr] == NO_INFORMATION) {
                                    ++no_inf_count;
                                }
                                if(no_inf_count > 2) {
                                    return true;
                                }
                            }
                        }
                    }

        return false;

    }

    bool FrontierSearch::nearestCell(unsigned int &result, unsigned int start, int val) {

        if (start >= size_x_ * size_y_) {
            return false;
        }

        //initialize breadth first search
        std::queue<unsigned int> bfs;
        std::vector<bool> visited_flag(size_x_ * size_y_, false);

        //push initial cell
        bfs.push(start);
        visited_flag[start] = true;

        //search for neighbouring cell matching value
        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            //return if cell of correct value is found
            if (map_data_[idx] == val) {
                result = idx;
                return true;
            }

            //iterate over all adjacent unvisited cells

            BOOST_FOREACH(unsigned int nbr, nhood8(idx, size_x_, size_y_)) {
                            if (!visited_flag[nbr]) {
                                bfs.push(nbr);
                                visited_flag[nbr] = true;
                            }
                        }
        }

        return false;
    }

    std::vector<unsigned int> FrontierSearch::nhood4(unsigned int idx, unsigned int width, unsigned int height) {
        //get 4-connected neighbourhood indexes, check for edge of map
        std::vector<unsigned int> out;

        unsigned int size_x_ = width, size_y_ = height;

        if (idx > size_x_ * size_y_ - 1) {
            ROS_WARN("Evaluating nhood for offmap point");
            return out;
        }

        if (idx % size_x_ > 0) {
            out.push_back(idx - 1);
        }
        if (idx % size_x_ < size_x_ - 1) {
            out.push_back(idx + 1);
        }
        if (idx >= size_x_) {
            out.push_back(idx - size_x_);
        }
        if (idx < size_x_ * (size_y_ - 1)) {
            out.push_back(idx + size_x_);
        }
        return out;
    }

    std::vector<unsigned int> FrontierSearch::nhood8(unsigned int idx, unsigned int width, unsigned int height) {

        //get 8-connected neighbourhood indexes, check for edge of map
        std::vector<unsigned int> out = nhood4(idx, width, height);

        unsigned int size_x_ = width, size_y_ = height;

        if (idx > size_x_ * size_y_ - 1) {
            ROS_WARN("Evaluating nhood for offmap point");
            return out;
        }

        if (idx % size_x_ > 0 && idx >= size_x_) {
            out.push_back(idx - 1 - size_x_);
        }
        if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
            out.push_back(idx - 1 + size_x_);
        }
        if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
            out.push_back(idx + 1 - size_x_);
        }
        if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
            out.push_back(idx + 1 + size_x_);
        }

        return out;

    }

    void FrontierSearch::indexToReal(const nav_msgs::OccupancyGrid &map, const size_t index, float &x, float &y) {
        const float xcenter = (map.info.width / 2) * map.info.resolution;
        const float ycenter = (map.info.height / 2) * map.info.resolution;
        const size_t row = index / map.info.height;
        const size_t col = index % map.info.width;
        const float xindex = col * map.info.resolution;
        const float yindex = row * map.info.resolution;
        x = xindex - xcenter;
        y = yindex - ycenter;
    }

}