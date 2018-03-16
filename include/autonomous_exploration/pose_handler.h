#ifndef POSE_HANDLER_H
#define POSE_HANDLER_H

#include <tf/transform_listener.h>

namespace tfhandler{

class PoseHandler
{
public:
  PoseHandler() {

  }

  tf::StampedTransform lookupPose(std::string parent="/map", std::string child="/base_link") {
      tf::StampedTransform transform;
      int temp0 = 0;
      // wait here until receive tf tree
      while (temp0 == 0) {
          try {
              temp0 = 1;
              listener.lookupTransform(parent, child, ros::Time(0), transform);
          } catch (tf::TransformException ex) {
              temp0 = 0;
              ros::Duration(0.1).sleep();
              ROS_WARN("no tf tree is received!");
          }
      }
      return transform;
  }

private:
    tf::TransformListener listener;

};

}
#endif // POSE_HANDLER_H
