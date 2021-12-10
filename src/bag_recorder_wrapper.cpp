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
#include "bag_recorder/futils.hpp"

using namespace std::chrono_literals;

pid_t PID;
bool recording(false);
char **argv_new;

void sigint_handler(int s){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Caught [%s] signal. Shutting down...\n", strsignal(s));

    // If the process is killed while a recording is in progress we need to cleanup
    if (recording){
        delete[] argv_new;
        kill(PID, 15);  //Sends the SIGINT Signal to the process, telling it to stop.
        std::cout << "Recording stopped." << std::endl;
    }

    rclcpp::shutdown();
}

void recorder_bag_handler(const std::shared_ptr<ulisse_msgs::srv::RosbagCmd::Request> request,
    std::shared_ptr<ulisse_msgs::srv::RosbagCmd::Response> response)
{

    // We start recording only if there is no actual recording
    if (request->record == 1) {
        if(recording == false) {
            std::string home_path = futils::get_homepath();
            std::string current_date = futils::GetCurrentDateFormatted();
            std::string sys_folder;

            if (!request->save_folder.empty()){
                sys_folder = request->save_folder;
            } else {
                sys_folder = home_path + "/logs/bag_recorder/";
            }

            std::string bag_folder = sys_folder + "rosbag2_" + current_date;


            if (futils::does_file_exists(sys_folder)) {
                recording = true;
                response->res = "Recording Started.";

                std::vector<std::string> args;
                args.push_back("bag");
                args.push_back("record");
                args.push_back("-a");
                args.push_back("-o");
                args.push_back(bag_folder);
                argv_new = new char*[args.size() + 2];

                argv_new[0] = (char*)"ros2";
                argv_new[args.size() + 1] = NULL;

                for(unsigned int c=0; c<args.size(); c++)
                    argv_new[c+1] = (char*)args[c].c_str();

                PID = fork();
                if(PID == 0) {
                    execvp(argv_new[0], argv_new);
                    perror("execvp");
                    exit(1);
                }

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bag Folder: %s", bag_folder.c_str());
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Recording started.");
            } else {
                response->res = "Save folder does not exist. No bag will be recorded.";
            }
        } else {
            response->res = "Recording already in progress.";
        }

    } else if (request->record == 0) {
        if (recording == true) {
            recording = false;
            response->res = "Recording Stopped.";
            delete[] argv_new;
            kill(PID, 15);  //Sends the SIGINT Signal to the process, telling it to stop.

            std::cout << "Recording stopped." << std::endl;
        } else {
            response->res = "No active record to stop. NOP.";
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->res.c_str());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = sigint_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("record_bag_server");
    rclcpp::Service<ulisse_msgs::srv::RosbagCmd>::SharedPtr trigger_srv =
        node->create_service<ulisse_msgs::srv::RosbagCmd>("record_bag_service", &recorder_bag_handler);
    RCLCPP_INFO(node->get_logger(), "Ready to record Bag.");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
