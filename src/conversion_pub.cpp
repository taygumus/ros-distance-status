#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class ConversionPub {

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher odom_pub;

    tf::TransformBroadcaster br;

    float lat_init = 0;
    float long_init = 0;
    float alt_init = 0;

    std::string child;

public:
    ConversionPub(std::string topic, std::string obj) {
        n.getParam("/conversion_pub/lat", lat_init);
        n.getParam("/conversion_pub/long", long_init);
        n.getParam("/conversion_pub/alt", alt_init);

        child = obj;
        sub = n.subscribe(topic, 1000, &ConversionPub::callback, this);
        odom_pub = n.advertise<nav_msgs::Odometry>(child, 1000);
    }

    void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // fixed values
        double a = 6378137;
        double b = 6356752.3142;
        double f = (a - b) / a;
        double e_sq = f * (2 - f);
        float deg_to_rad = 0.0174533;

        // input data from msg
        float latitude = msg->latitude;
        float longitude = msg->longitude;
        float h = msg->altitude;

        // fixed position from .launch file
        float latitude_init = lat_init;
        float longitude_init = long_init;
        float h0 = alt_init;

        // lla to ecef
        float lamb = deg_to_rad * (latitude);
        float phi = deg_to_rad * (longitude);
        float s = sin(lamb);
        float N = a / sqrt(1 - e_sq * s * s);

        float sin_lambda = sin(lamb);
        float cos_lambda = cos(lamb);
        float sin_phi = sin(phi);
        float cos_phi = cos(phi);

        float x = (h + N) * cos_lambda * cos_phi;
        float y = (h + N) * cos_lambda * sin_phi;
        float z = (h + (1 - e_sq) * N) * sin_lambda;

        // ecef to enu
        lamb = deg_to_rad * (latitude_init);
        phi = deg_to_rad * (longitude_init);
        s = sin(lamb);
        N = a / sqrt(1 - e_sq * s * s);

        sin_lambda = sin(lamb);
        cos_lambda = cos(lamb);
        sin_phi = sin(phi);
        cos_phi = cos(phi);

        float x0 = (h0 + N) * cos_lambda * cos_phi;
        float y0 = (h0 + N) * cos_lambda * sin_phi;
        float z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

        float xd = x - x0;
        float yd = y - y0;
        float zd = z - z0;

        float xEast = -sin_phi * xd + cos_phi * yd;
        float yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
        float zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

        ROS_INFO("Position: [%f,%f, %f]", xEast, yNorth, zUp);

        // better visualization in rviz
        float scale = 100.0;

        xEast /= scale;
        yNorth /= scale;
        zUp /= scale;

        // tf
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(xEast, yNorth, zUp));

        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", child));

        // odom
        nav_msgs::Odometry odom;

        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world";

        odom.pose.pose.position.x = xEast;
        odom.pose.pose.position.y = yNorth;
        odom.pose.pose.position.z = zUp;

        odom_pub.publish(odom);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "subscribe_convert_publish");

    ConversionPub car_tf("/swiftnav/front/gps_pose", "car");
    ConversionPub obs_tf("/swiftnav/obs/gps_pose", "obstacle");

    ros::spin();
    return 0;
}
