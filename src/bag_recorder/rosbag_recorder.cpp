#include "bag_recorder/rosbag_recorder.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

RosbagRecorder::RosbagRecorder() :
    Node("rosbag_recorder"),
    recording(false)
{

    trigger_srv_ = this->create_service<ulisse_msgs::srv::RosbagCmd>("record_bag_service", std::bind(&RosbagRecorder::ServiceHandler, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Ready to record Bag.");
}

RosbagRecorder::~RosbagRecorder()
{
    // If the process is killed while a recording is in progress we need to cleanup
    if (recording){
        delete[] argv_new;
        kill(PID, 15);  //Sends the SIGINT Signal to the process, telling it to stop.
        RCLCPP_INFO(this->get_logger(), "Recording stopped.");
    }

}

void RosbagRecorder::ServiceHandler(const std::shared_ptr<ulisse_msgs::srv::RosbagCmd::Request> request,
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
