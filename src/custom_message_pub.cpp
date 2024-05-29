#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_project/Status.h"
#include "ros_project/ComputeDistance.h"
#include <time.h>
#include <dynamic_reconfigure/server.h>
#include <ros_project/parametersConfig.h>

class CustomMessagePub {

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Timer timer;
    ros::ServiceClient request;

    ros_project::ComputeDistance srv;

    int safe_distance;

    dynamic_reconfigure::Server<ros_project::parametersConfig> server;
    dynamic_reconfigure::Server<ros_project::parametersConfig>::CallbackType f;

public:
    CustomMessagePub(bool use_dynamic_reconfigure) {
        n.getParam("/custom_message_pub/safe_distance", safe_distance);

        timer = n.createTimer(ros::Duration(2), &CustomMessagePub::timer_callback, this);
        request = n.serviceClient<ros_project::ComputeDistance>("serve_distance");
        pub = n.advertise<ros_project::Status>("status", 1000);

        if (use_dynamic_reconfigure == true) {
            f = boost::bind(&CustomMessagePub::dynamic_reconfigure_callback, this, _1);
            server.setCallback(f);
        }
    }

    void timer_callback(const ros::TimerEvent& ev) {
        if (request.call(srv)) {
            double distance = (double) srv.response.distance;

            if (!std::isnan(distance)) {
                // message building
                ros_project::Status msg;
                msg.distance = distance;

                if (distance > safe_distance) {
                    msg.status = "Safe";
                }
                else if (distance <= safe_distance && distance >= 1) {
                    msg.status = "Unsafe";
                }
                else if (distance < 1) {
                    msg.status = "Crash";
                }

                pub.publish(msg);

                ROS_INFO(" ");
                std::cout << "Distance: " << msg.distance << " - " << "Status: " << msg.status << "\n";
            }
            else {
                ROS_INFO("Distance is nan");
            }
        }
        else {
            ROS_ERROR("Failed to call service serve_distance");
            return;
        }
    }

    void dynamic_reconfigure_callback(ros_project::parametersConfig &config) {
        safe_distance = config.safe_distance;
    }
};

int main(int argc, char **argv) {   
    ros::init(argc, argv, "custom_message_pub");

    CustomMessagePub message_pub(true);

    ros::spin();
    return 0;
}
