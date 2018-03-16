#ifndef POSE_WRAP_H
#define POSE_WRAP_H

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>

using namespace ros;

class PoseWrap
{
public:
	PoseWrap() {

	}

	PoseWrap& operator = (const PoseWrap &rhs) {
		pose.position.x = x = rhs.x;
		pose.position.y = y = rhs.y;
		pose.orientation.w = theta = rhs.theta;
        return *this;
	}

	PoseWrap(const PoseWrap &t) {
		pose.position.x = x = t.x;
		pose.position.y = y = t.y;
		pose.orientation.w = theta = t.theta;
	}

	PoseWrap(double a, double b, double c = 1.) : x(a), y(b), theta(c){
		pose.position.x = a;
		pose.position.y = b;
		pose.orientation.w = c;
	}

	geometry_msgs::Pose getPose() {
		return pose;
	}

	double x, y , theta;

protected:
	geometry_msgs::Pose pose;
};

#endif

