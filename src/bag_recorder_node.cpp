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

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "ulisse_msgs/srv/rosbag_cmd.hpp"
#include "bag_recorder/rosbag_recorder.hpp"
#include "bag_recorder/futils.hpp"

using namespace std::chrono_literals;


int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);

    auto rosbag_recorder = std::make_shared<RosbagRecorder>();

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(rosbag_recorder);
    exe.spin();

    rclcpp::shutdown();

    return 0;
}
