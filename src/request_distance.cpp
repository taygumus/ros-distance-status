#include "ros/ros.h"
#include "ros_project/ComputeDistance.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "request_distance");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ros_project::ComputeDistance>("serve_distance");

    ros_project::ComputeDistance srv;

    if (client.call(srv)) {
      ROS_INFO("Distance in meters between the two sources: %f", (double) srv.response.distance);
    }
    else {
      ROS_ERROR("Failed to call service serve_distance");
      return 1;
    }

    return 0;
}
