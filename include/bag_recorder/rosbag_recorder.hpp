#ifndef ROSBAGRECORDER_HPP
#define ROSBAGRECORDER_HPP

#include <cstdio>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <sys/wait.h>
#include <chrono>
#include <thread>
#include <memory>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "ulisse_msgs/srv/rosbag_cmd.hpp"
#include "bag_recorder/futils.hpp"



class RosbagRecorder : public rclcpp::Node
{
    pid_t PID;
    bool recording;
    char **argv_new;
    rclcpp::Service<ulisse_msgs::srv::RosbagCmd>::SharedPtr trigger_srv_;

    void ServiceHandler(const std::shared_ptr<ulisse_msgs::srv::RosbagCmd::Request> request,
        std::shared_ptr<ulisse_msgs::srv::RosbagCmd::Response> response);
public:
    RosbagRecorder();

    virtual ~RosbagRecorder();
};

#endif // ROSBAGRECORDER_HPP
