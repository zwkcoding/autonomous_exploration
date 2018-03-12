//
// Created by kevin on 3/10/18.
//

#ifndef AUTONOMOUS_EXPLORATION_UTIL_HPP
#define AUTONOMOUS_EXPLORATION_UTIL_HPP

namespace util {
    inline double calcDistance(double x1, double y1, double x2, double y2)
    {
        return std::hypot(x2 - x1, y2 - y1);  // avoid overflow or underflow at intermediate stages
        //return std::sqrt((x2 - x1)*(x2-x1) + (y2 - y1)*(y2 - y1));
    }

    inline double modifyTheta(double theta)
    {
        if (theta < 0)
            return theta + 2 * M_PI;
        if (theta >= 2 * M_PI)
            return theta - 2 * M_PI;

        return theta;
    }

    inline double calcDiffOfRadian(double a, double b)
    {
        double diff = std::fabs(a - b);
        if (diff < M_PI)
            return diff;
        else
            return 2 * M_PI - diff;
    }
}

#endif //AUTONOMOUS_EXPLORATION_UTIL_HPP
