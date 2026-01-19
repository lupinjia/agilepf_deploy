#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <math.h>
#include <iostream>
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

#include "common/mathTools.h"
#include "common/periodic_task.hpp"
#include <idl/agilepf/LowState/LowState.hpp>
#include <idl/agilepf/LowState/LowStatePubSubTypes.hpp>
#include <dds/dds_publisher.hpp>
#include <dds/dds_subscriber.hpp>
#include <idl/agilepf/LowCmd/LowCmd.hpp>
#include <idl/agilepf/LowCmd/LowCmdPubSubTypes.hpp>

#include "state_machine.hpp"
#include "robot_interface.hpp"
#include "gamepad.hpp"
#include "user_controller.hpp"

namespace fs = std::filesystem;
using namespace legged::dds;
using namespace legged::common;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

// RobotController为一个框架, 将有限状态机整体的逻辑具象化为一个类
class RobotController
{
public:
    RobotController(BasicUserController* ctrl) : ctrl(ctrl) {}

    RobotController(fs::path &log_file_name, BasicUserController* user_ctrl,
                    int domain_id, std::string interface_name) :
        ctrl(user_ctrl), domain_id_(domain_id), interface_name_(interface_name)
    {
        // set log file
        log_file = std::ofstream(log_file_name);
        std::stringstream header;
        header << "compute_time,";
        header << "state,";
        header << "cmd_lin_x,cmd_lin_y,cmd_ang_z,";                            // cmd
        header << "projected_gravity0,projected_gravity1,projected_gravity2,"; // projected_gravity
        header << "base_ang_vel_x,base_ang_vel_y,base_ang_vel_z,";             // base_ang_vel
        header << "jpos_0,jpos_1,jpos_2,jpos_3,jpos_4,jpos_5,";                // dof_pos
        header << "jvel_0,jvel_1,jvel_2,jvel_3,jvel_4,jvel_5,";                // dof_vel
        header << "clock_input0,clock_input1,clock_input2,clock_input3,";   // clock_input
        header << "gait_time,gait_phase,";
        // header << "last_action0,last_action1,last_action2,last_action3,last_action4,last_action5,";          // actions
        // header << "action0,action1,action2,action3,action4,action5,"; // actions
        // header << "kp_0,kp_1,kp_2,kp_3,kp_4,kp_5,";
        // header << "kd_0,kd_1,kd_2,kd_3,kd_4,kd_5";
        // header << "jpos_des0,jpos_des1,jpos_des2,jpos_des3,jpos_des4,jpos_des5,"; // last_jpos_des
        // header << "jvel_des0,jvel_des1,jvel_des2,jvel_des3,jvel_des4,jvel_des5,"; // jvel_des
        // header << "tau0,tau1,tau2,tau3,tau4,tau5,"; // tau_ff
        header << std::endl;
        log_file << header.str();

        initLowCmd();
    }

    void loadParam()
    {
        ctrl->loadParam();
    }

    void loadPolicy()
    {
        ctrl->loadPolicy();
    }

    void initDDSPubSub()
    {
        low_cmd_publisher = std::make_shared<DDSPublisher<legged::idl::agilepf::LowCmd,
                        legged::idl::agilepf::LowCmdPubSubType>>(domain_id_, interface_name_, TOPIC_LOWCMD, 2000.0f);
        low_state_subscriber = std::make_shared<DDSSubscriber<legged::idl::agilepf::LowState,
                         legged::idl::agilepf::LowStatePubSubType>>(domain_id_, interface_name_, TOPIC_LOWSTATE, 
                            std::bind(&RobotController::lowStateMessageHandler, this, std::placeholders::_1), 2000.0f);
        
        low_cmd_publisher->init();
        low_state_subscriber->init();
    }

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
            -1, SCHED_FIFO, 90);

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

    std::ofstream log_file;

    Gamepad gamepad;

    uint64_t ctrl_dt_micro_sec = 2000;

    std::vector<float> compute_time;

    int domain_id_;
    std::string interface_name_;

private:
        
    // 500Hz
    void lowStateMessageHandler(const void* msg)
    {
        low_state = *(legged::idl::agilepf::LowState*)msg;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            robot_interface.updateLowState(low_state);
        }
    }

    void intepretGamePad()
    {
        gamepad.update(low_state.joystick_data());
    }

    // 500Hz
    void lowCmdwriteHandler()
    {
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            low_cmd_publisher->publish(&low_cmd);
        }
    }

    void initLowCmd()
    {
        // initialize low_cmd msg
        low_cmd.header() = {0xAA, 0xBB};
        low_cmd.tail() = {0xEE, 0xFF};
    }

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

    void startSendCmd()
    {
        // write_low_cmd thread: 2000us(2ms), 500Hz; running on CPU4
        low_cmd_publish_task = createPeriodicTask("write_low_cmd", 2000, 
            std::bind(&RobotController::lowCmdwriteHandler, this),
            4, SCHED_FIFO, 99);
    }

   void updateStateMachine()
    {
        // B -> Stand
        // A -> CTRL
        // Y -> Stop
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

    // 50Hz
    void controlStep()
    {
        intepretGamePad();
        // update current state
        updateStateMachine();

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
            std::cout << "control thread Performance: mean: " << sum / 50
                      << " ms; max: " << *std::max_element(compute_time.begin(), compute_time.end())
                      << " ms; min: " << *std::min_element(compute_time.begin(), compute_time.end())
                      << "ms." << std::endl;

            compute_time.clear();

            std::cout << "Current State: " << state_machine.state_name[static_cast<size_t>(state_machine.state)] << std::endl;
        }
    }

    void writeLog(float computation_time)
    {
        if (log_file.is_open())
        {
            log_file << computation_time << ",";
            log_file << static_cast<size_t>(state_machine.state) << ",";
            auto log = ctrl->getLog();
            for (const auto &v : log)
            {
                log_file << v << ",";
            }

            log_file << std::endl;
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
        ctrl->dummyCalculate(); // warm up the neural network
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
        // if (robot_interface.projected_gravity.at(2) > -0.5)
        // {
        //     std::cout << "Termination: projected_gravity.z = " << robot_interface.projected_gravity.at(2) << std::endl;
        //     return true;
        // }
        return false;
    }
};

#endif