#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <optional>
#include <chrono>
#include <iomanip>
#include <thread>

#include "robot_controller.hpp"
#include "robot_interface.hpp"

namespace fs = std::filesystem;

int main(int argc, char const *argv[])
{
    // constant param_folder
    std::string param_folder = "../params";
    fs::path param = fs::current_path() / param_folder;

    /***** log *****/
    // get the current time
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y%m%d-%H%M%S");
    // create log folder named by current time
    fs::path log_folder = fs::current_path() / "logs" / ss.str();
    fs::create_directories(log_folder);
    // using csv format to save log data
    fs::path log_file_name = log_folder / "log.csv";

    /***** Controller *****/
    std::string config_file_dir = (param / "policy_params.yaml").string();
    OnnxController* user_ctrl = new OnnxController(config_file_dir);
    
    int domain_id;
    std::string interface_name;
    if(argc > 1) // use specified interface name
    {
        domain_id = 0;
        interface_name = argv[1];
    }
    else // local loopback
    {
        domain_id = 1;
        interface_name = "lo";
    }
    RobotController *robot_controller = new RobotController(log_folder, 
                                    user_ctrl, domain_id, interface_name);
    // load parameter from .json file
    robot_controller->loadParam();
    // 加载神经网络模型
    robot_controller->loadPolicy();

    // output information
    spdlog::info("WARNING: Make sure the robot is hung up or lying on the ground.");
    spdlog::info("Press Enter to continue...");
    std::cin.ignore();

    // 开启外设通信
    robot_controller->initDDSPubSub();
    std::chrono::milliseconds duration(1000); // 1000ms, wait for peripheral threads to start
    std::this_thread::sleep_for(duration);

    // start control loop
    robot_controller->startControl();

    return 0;
}
