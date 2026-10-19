/**
 * @file robot_controller.hpp
 * @author lupinjia (jason_1120202397@163.com)
 * @brief RobotController class. This class is the outside interface that incorporates every module of the framework,
 *        including state machine, robot interface, gamepad, user controller, and DDS communication.
 *        The class is responsible for initializing the framework, loading parameters, and running the robot control loop.
 * @version 0.1
 * @date 2026-10-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <filesystem>
#include <mutex>
#include <fstream>
#include <future>
#include <algorithm>
#include <sstream>
#include <sched.h>
#include <typeinfo>
#include <exception>

#include "spdlog/sinks/basic_file_sink.h"
#include "common/mathTools.h"
#include <legged_sdk/common/periodic_task.hpp>
#include <legged_sdk/idl/agilepf/LowState/LowState.hpp>
#include <legged_sdk/idl/agilepf/LowState/LowStatePubSubTypes.hpp>
#include <legged_sdk/dds/dds_publisher.hpp>
#include <legged_sdk/dds/dds_subscriber.hpp>
#include <legged_sdk/idl/agilepf/LowCmd/LowCmd.hpp>
#include <legged_sdk/idl/agilepf/LowCmd/LowCmdPubSubTypes.hpp>

#include "state_machine.hpp"
#include "robot_interface.hpp"
#include "gamepad.hpp"
#include "user_controller.hpp"

namespace fs = std::filesystem;
using namespace legged::dds;
using namespace legged::common;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

/**
 * @brief RobotController class.
 * 
 */
class RobotController
{
public:
    /**
     * @brief Construct a new RobotController object.
     * 
     * @param ctrl User controller.
     */
    RobotController(BasicUserController* ctrl) : ctrl(ctrl) {}

    /**
     * @brief Construct a new RobotController object.
     * 
     * @param log_folder Path of log folder.
     * @param user_ctrl User controller.
     * @param domain_id Domain ID.
     * @param interface_name Interface name.
     */
    RobotController(fs::path &log_folder, BasicUserController* user_ctrl,
                    int domain_id, std::string interface_name) :
        ctrl(user_ctrl), domain_id_(domain_id), interface_name_(interface_name)
    {
        // set log file
        robot_state_log_file = std::ofstream(log_folder / "robot_state_log.csv");
        std::stringstream header;
        header << "compute_time,";
        header << "state,";
        header << "cmd_lin_x,cmd_lin_y,cmd_ang_z,"; // cmd (3)
        header << "projected_gravity_x,projected_gravity_y,projected_gravity_z,"; // gravity (3)
        header << "gyro_x,gyro_y,gyro_z,"; // gyro (3)
        header << "joint_position_0,joint_position_1,joint_position_2,joint_position_3,joint_position_4,joint_position_5,"; // joint position (6)
        header << "joint_velocity_0,joint_velocity_1,joint_velocity_2,joint_velocity_3,joint_velocity_4,joint_velocity_5,"; // joint velocity (6)
        header << "action_0,action_1,action_2,action_3,action_4,action_5"; // actions (6)
        header << std::endl;
        robot_state_log_file << header.str();

        // initialize spdlog logger
        message_logger = spdlog::basic_logger_mt("message_logger", log_folder / "message_log.txt");
        // spdlog::set_default_logger(logger);

        // initialize lowlevel command
        initLowCmd();
    }

    /**
     * @brief Load parameter from yaml file.
     * 
     */
    void loadParam()
    {
        ctrl->loadParam();
    }

    /**
     * @brief Load NN policy.
     * 
     */
    void loadPolicy()
    {
        ctrl->loadPolicy();
    }

    /**
     * @brief Initialize DDS communication.
     * 
     */
    void initDDSPubSub()
    {
        // init time variables
        low_state_last_time = std::chrono::high_resolution_clock::now();

        // init DDS publisher and subscriber
        low_cmd_publisher = std::make_shared<DDSPublisher<legged::idl::agilepf::LowCmd,
                        legged::idl::agilepf::LowCmdPubSubType>>(domain_id_, interface_name_, TOPIC_LOWCMD, 2000.0f);
        low_state_subscriber = std::make_shared<DDSSubscriber<legged::idl::agilepf::LowState,
                         legged::idl::agilepf::LowStatePubSubType>>(domain_id_, interface_name_, TOPIC_LOWSTATE, 
                            std::bind(&RobotController::lowStateMessageHandler, this, std::placeholders::_1), 2000.0f);
        low_cmd_publisher->init();
        low_state_subscriber->init();
        
    }

    /**
     * @brief Start control thread.
     * 
     */
    void startControl()
    {
        // waiting for gamepad command to start the control thread
        std::chrono::milliseconds duration(100); // 100ms

        // prepare for start
        spdlog::info("Start!");
        zeroTorque();
        ctrl_dt_micro_sec = static_cast<uint64_t>(ctrl->dt * 1000000); // 0.02s, 50Hz
        // control thread: 20000us(20ms), 50Hz; running on CPU3
        control_task = createPeriodicTask("control_thread", ctrl_dt_micro_sec, 
            std::bind(&RobotController::controlStep, this),
            3, SCHED_FIFO, 90);

        // Start the lowlevel command thread
        std::this_thread::sleep_for(duration);
        startSendCmd();
        spdlog::info("Start Send Cmd");

        // keep the main thread alive
        while (true)
        {
            std::this_thread::sleep_for(duration);
        }
    }

protected:
    DDSPublisherPtr<legged::idl::agilepf::LowCmd,
                        legged::idl::agilepf::LowCmdPubSubType> low_cmd_publisher;
    DDSSubscriberPtr<legged::idl::agilepf::LowState,
                         legged::idl::agilepf::LowStatePubSubType> low_state_subscriber;
    PeriodicTaskPtr low_cmd_publish_task, control_task;

    legged::idl::agilepf::LowCmd low_cmd;
    legged::idl::agilepf::LowState low_state;

    SimpleStateMachine state_machine;
    BasicUserController* ctrl;
    BasicRobotInterface robot_interface;

    std::mutex state_mutex, cmd_mutex;

    std::ofstream robot_state_log_file;
    std::shared_ptr<spdlog::logger> message_logger;

    Gamepad gamepad;

    uint64_t ctrl_dt_micro_sec = 2000;

    // time variables
    std::chrono::high_resolution_clock::time_point low_state_last_time;
    std::vector<float> low_state_period;
    std::chrono::high_resolution_clock::time_point low_cmd_last_time;
    std::vector<float> low_cmd_period;
    std::vector<float> compute_time;
    bool control_fault_latched = false;

    int domain_id_;
    std::string interface_name_;

private:
        
    /**
     * @brief Handle low state message. This function is executed at 500Hz.
     * 
     * @param msg Low state message.
     */
    void lowStateMessageHandler(const void* msg)
    {
        low_state = *(legged::idl::agilepf::LowState*)msg;
        #ifdef PRINT_DEBUG
            // calculate time between two successive messages
            auto cur_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(cur_time - low_state_last_time).count();
            low_state_last_time = cur_time;
            low_state_period.push_back(duration / 1000.0f); // ms
            if (low_state_period.size() == 500)
            {
                float sum = 0;
                float min = low_state_period[0];
                float max = 0;
                for (auto &t : low_state_period)
                {
                    sum += t;
                    if( t < min ) min = t;
                    if( t > max ) max = t;
                }
                message_logger->info("low_state subscriber Performance: mean: {} ms; max: {} ms; min: {} ms.",
                            sum / 500, max, min);
                message_logger->info("low_state motor position: {}, {}, {}, {}, {}, {}.",
                            low_state.motor_state()[0].q(),
                            low_state.motor_state()[1].q(),
                            low_state.motor_state()[2].q(),
                            low_state.motor_state()[3].q(),
                            low_state.motor_state()[4].q(),
                            low_state.motor_state()[5].q());
                message_logger->info("low_state subscriber cpu id: {}.",
                            sched_getcpu());
                message_logger->flush();
                low_state_period.clear();
            }
        #endif
        // process low state message
        {
            // lock state_mutex
            std::lock_guard<std::mutex> lock(state_mutex);
            robot_interface.updateLowState(low_state);
        }
    }

    /**
     * @brief Interpret gamepad data.
     * 
     */
    void intepretGamePad()
    {
        gamepad.update(low_state.joystick_data());
    }
    /**
     * @brief Handle low command message. This function is executed at 500Hz.
     * 
     */
    void lowCmdwriteHandler()
    {
        #ifdef PRINT_DEBUG
            // calculate time between two successive messages
            auto cur_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(cur_time - low_cmd_last_time).count();
            low_cmd_last_time = cur_time;
            low_cmd_period.push_back(duration / 1000.0f); // ms
            if (low_cmd_period.size() == 500)
            {
                float sum = 0;
                float min = low_cmd_period[0];
                float max = 0;
                for (auto &t : low_cmd_period)
                {
                    sum += t;
                    if( t < min ) min = t;
                    if( t > max ) max = t;
                }
                message_logger->info("low_cmd publisher Performance: mean: {} ms; max: {} ms; min: {} ms.",
                            sum / 500, max, min);
                message_logger->info("low_cmd publisher cpu id: {}.",
                            sched_getcpu());
                message_logger->flush();
                low_cmd_period.clear();
            }
        #endif
        // publish low level command
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            low_cmd_publisher->publish(&low_cmd);
        }
    }

    /**
     * @brief Initialize low level command.
     * 
     */
    void initLowCmd()
    {
        // initialize low_cmd msg
        low_cmd.header() = {0xAA, 0xBB};
        low_cmd.tail() = {0xEE, 0xFF};
    }

    /**
     * @brief Set low level command. Using the value in robot interface.
     * 
     */
    void set_cmd()
    {
        for(int i = 0; i < 6; i++)
        {
            low_cmd.motor_cmd()[i].q() = robot_interface.jpos_des.at(i);
            low_cmd.motor_cmd()[i].dq() = robot_interface.jvel_des.at(i);
            low_cmd.motor_cmd()[i].kp() = robot_interface.kp.at(i);
            low_cmd.motor_cmd()[i].kd() = robot_interface.kd.at(i);
            low_cmd.motor_cmd()[i].tau() = robot_interface.tau_ff.at(i);
        }
    }

    /**
     * @brief Start to send low level command.
     * 
     */
    void startSendCmd()
    {
        // save starting time
        low_cmd_last_time = std::chrono::high_resolution_clock::now();
        // write_low_cmd thread: 2000us(2ms), 500Hz; running on CPU4
        low_cmd_publish_task = createPeriodicTask("write_low_cmd", 2000, 
            std::bind(&RobotController::lowCmdwriteHandler, this),
            4, SCHED_FIFO, 99);
    }

    /**
     * @brief Update state machine.
     * 
     */
    void updateStateMachine()
    {
        // B -> Stand
        // A -> CTRL
        // Y -> Stop
        // X -> Zero Torque
        if(gamepad.B.on_press)
        {
            if(state_machine.stand())
            {
                standCallback();
            }
        }
        if(gamepad.A.on_press)
        {
            if(state_machine.ctrl())
            {
                ctrlCallback();
            }
        }
        if(gamepad.Y.on_press)
        {
            state_machine.damp();
        }
        if(gamepad.X.on_press)
        {
            state_machine.zeroTorque();
        }
    }

    /**
     * @brief Control step. This function is executed at 50Hz.
     * 
     */
    void controlStep()
    {
      try
      {
        // interpret gamepad data at 50Hz, avoiding multiple presses
        intepretGamePad();
        // update current state
        updateStateMachine();
        // update imu offset in robot interface
        if(gamepad.left.on_press) // left and right button to adjust roll offset
        {
            robot_interface.rpy_offset.at(0) += ctrl->rpy_offset_increment;
        }
        if(gamepad.right.on_press)
        {
            robot_interface.rpy_offset.at(0) -= ctrl->rpy_offset_increment;
        }
        if(gamepad.up.on_press) // up and down button to adjust pitch offset
        {
            robot_interface.rpy_offset.at(1) -= ctrl->rpy_offset_increment;
        }
        if(gamepad.down.on_press)
        {
            robot_interface.rpy_offset.at(1) += ctrl->rpy_offset_increment;
        }

        // select control modes according to the state machine
        auto start = std::chrono::high_resolution_clock::now();
        // 读取状态信息
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            ctrl->getInput(robot_interface, gamepad);
        }
        
        // 根据状态判断具体执行哪个状态下的处理函数
        // 
        if(state_machine.state == STATES::STAND)
        {
            standing();
        }
        if(state_machine.state == STATES::DAMPING)
        {
            damping();
        }
        if(state_machine.state == STATES::ZERO_TORQUE)
        {
            zeroTorque();
        }
        if(state_machine.state == STATES::CTRL)
        {
            userControlStep(true); // send command
            if (CheckTermination())
            {
                state_machine.damp();
                damping();
            }
        }

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        compute_time.push_back(duration.count() / 1000.); // us->ms

        // update low-level command
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            set_cmd();
        }

        // write log
        writeLog(duration.count() / 1000.);

        if (compute_time.size() == 50)
        {
            float sum = 0;
            for (auto &t : compute_time)
            {
                sum += t;
            }
            spdlog::info("control thread Performance: mean: {} ms; max: {} ms; min: {} ms.",
                         sum / 50,
                         *std::max_element(compute_time.begin(), compute_time.end()),
                         *std::min_element(compute_time.begin(), compute_time.end()));
            spdlog::info("Current IMU rpy offset: roll: {}, pitch: {}, yaw: {}",
                     robot_interface.rpy_offset.at(0),
                     robot_interface.rpy_offset.at(1),
                     robot_interface.rpy_offset.at(2));
            #ifdef PRINT_DEBUG
            message_logger->info("control thread cpu id: {}.", sched_getcpu());
            message_logger->info("control thread Performance: mean: {} ms; max: {} ms; min: {} ms.",
                                sum / 50,
                                *std::max_element(compute_time.begin(), compute_time.end()),
                                *std::min_element(compute_time.begin(), compute_time.end()));
            message_logger->flush();
            #endif
            compute_time.clear();
            
            spdlog::info("Current State: {}", state_machine.state_name[static_cast<size_t>(state_machine.state)]);
        }
      }
      catch(const std::exception& error)
      {
        tripControlFault(error.what());
      }
      catch(...)
      {
        tripControlFault("unknown exception");
      }
    }

    void tripControlFault(const char* reason) noexcept
    {
        const bool first_fault = !control_fault_latched;
        control_fault_latched = true;
        state_machine.latchFault();

        // Atomically replace the active output before the publisher can resend it.
        robot_interface.jpos_des.fill(0.);
        robot_interface.jvel_des.fill(0.);
        robot_interface.kp.fill(0.);
        robot_interface.kd.fill(0.);
        robot_interface.tau_ff.fill(0.);
        try
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            set_cmd();
        }
        catch(...)
        {
            // Do not allow fault handling to escape the periodic callback.
        }

        if(first_fault)
        {
            try
            {
                spdlog::error("Control fault latched; entering ZERO_TORQUE: {}", reason);
            }
            catch(...)
            {
                // Logging must never interfere with the emergency command.
            }
        }
    }

    void writeLog(float computation_time)
    {
        if (robot_state_log_file.is_open())
        {
            robot_state_log_file << computation_time << ",";
            robot_state_log_file << static_cast<size_t>(state_machine.state) << ",";
            auto log = ctrl->getLog();
            for (const auto &v : log)
            {
                robot_state_log_file << v << ",";
            }

            robot_state_log_file << std::endl;
        }
    }

    /**
     * @brief 站立状态回调函数
     * @note  进入状态时调用一次, 用于进入站立状态的初始化工作
    */
    void standCallback()
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            ctrl->save_jpos(robot_interface);
        }
        robot_interface.jpos_des = ctrl->start_pos;
        robot_interface.jvel_des.fill(0.);
        robot_interface.kp.fill(ctrl->stand_kp);
        robot_interface.kd.fill(ctrl->stand_kd);
        robot_interface.tau_ff.fill(0.);
    }

    /**
        * @brief 控制状态回调函数
        * @note  进入状态时调用一次, 用于进入控制状态的初始化工作
    */
    void ctrlCallback()
    {
        ctrl->reset(robot_interface);

        robot_interface.jpos_des = ctrl->stand_pos;
        robot_interface.jvel_des.fill(0.);
        robot_interface.kp.fill(ctrl->ctrl_kp);
        robot_interface.kd.fill(ctrl->ctrl_kd);
        robot_interface.tau_ff.fill(0.);

    }

    /**
    * @brief processing function for DAMPING state
    * 
    */
    void damping(float kd = 3.0)
    {
        ctrl->dummyCalculate(); // warm up the neural network
        robot_interface.jpos_des.fill(0.);
        robot_interface.jvel_des.fill(0.);
        robot_interface.kp.fill(0.);
        robot_interface.kd.fill(kd);
        robot_interface.tau_ff.fill(0.);
    }

    void zeroTorque()
    {
        robot_interface.jpos_des.fill(0.);
        robot_interface.jvel_des.fill(0.);
        robot_interface.kp.fill(0.);
        robot_interface.kd.fill(0.);
        robot_interface.tau_ff.fill(0.);
    }

    void standing()
    {
        state_machine.standing(*ctrl);
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            robot_interface.jpos_des = ctrl->jpos_des;
        }
    }

    void userControlStep(bool send_cmd = false)
    {
        ctrl->calculate();

        if(send_cmd)
        { 
            {
                std::lock_guard<std::mutex> lock(cmd_mutex);
                robot_interface.jpos_des = ctrl->jpos_des;
            } 
        }
    }

    bool CheckTermination()
    {
        // if (robot_interface.projected_gravity.at(2) > -0.2)
        // {
        //     spdlog::info("Termination: projected_gravity.z = {}", robot_interface.projected_gravity.at(2));
        //     return true;
        // }
        return false;
    }
};

#endif
