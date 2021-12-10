#ifndef ROSBAGRECORDER_HPP
#define ROSBAGRECORDER_HPP

#include "rclcpp/rclcpp.hpp"

class RosbagRecorder : public rclcpp::Node
{
public:
    RosbagRecorder();

    virtual ~RosbagRecorder();
};

#endif // ROSBAGRECORDER_HPP
