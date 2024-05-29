#include "ros/ros.h"
#include "ros_project/ComputeDistance.h"
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class DistanceServer {

private:
    ros::NodeHandle n;
    ros::ServiceServer service;

    typedef message_filters::Subscriber<nav_msgs::Odometry> OdometryFilter;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;

    message_filters::Subscriber<nav_msgs::Odometry> *sub1;
    message_filters::Subscriber<nav_msgs::Odometry> *sub2;
    message_filters::Synchronizer<MySyncPolicy> *sync;

    nav_msgs::Odometry msg1;
    nav_msgs::Odometry msg2;

    float time_margin;
    float distance_scale;

public:
    DistanceServer(std::string first_topic, std::string second_topic) {
        // custom margin in seconds
        time_margin = 2.0;

        // scale value for distance
        distance_scale = 100.0;

        service = n.advertiseService("serve_distance", &DistanceServer::serve_distance, this);

        sub1 = new OdometryFilter(n, first_topic, 1);
        sub2 = new OdometryFilter(n, second_topic, 1);

        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *sub1, *sub2);
        sync->registerCallback(boost::bind(&DistanceServer::filter_messages, this, _1, _2));

        ROS_INFO("Ready to serve the distance between the two sources");
    }

    void filter_messages(const nav_msgs::OdometryConstPtr& first_msg, const nav_msgs::OdometryConstPtr& second_msg) {
        msg1 = *first_msg;
        msg2 = *second_msg;
    }

    bool serve_distance(ros_project::ComputeDistance::Request &req, ros_project::ComputeDistance::Response &res) {
        ros::Time last_msg1_time = msg1.header.stamp;
        ros::Time last_msg2_time = msg2.header.stamp;

        if (last_msg1_time.toSec() != 0 && last_msg2_time.toSec() != 0) {
            ros::Time now = ros::Time::now();
            ros::Duration difference = now - last_msg1_time;

            // compare between current time and last message time to verify GPS lost
            if (difference.toSec() <= time_margin) {
                float x1 = msg1.pose.pose.position.x * distance_scale;
                float y1 = msg1.pose.pose.position.y * distance_scale;
                float z1 = msg1.pose.pose.position.z * distance_scale;

                float x2 = msg2.pose.pose.position.x * distance_scale;
                float y2 = msg2.pose.pose.position.y * distance_scale;
                float z2 = msg2.pose.pose.position.z * distance_scale;

                res.distance = get_euclidean_distance(x1, y1, z1, x2, y2, z2);
            }
            else {
                res.distance = std::numeric_limits<double>::quiet_NaN();
                ROS_INFO("GPS of one or both sources is lost.");
            }
        }
        else {
            res.distance = std::numeric_limits<double>::quiet_NaN();
            ROS_INFO("No source data available.");
        }

        return true;
    }

    float get_euclidean_distance(float x1, float y1, float z1, float x2, float y2, float z2) {
        float distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2) * 1.0);
        return distance;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "serve_distance");

    DistanceServer server("car", "obstacle");

    ros::spin();
    return 0;
}
