/**
 * @file      user_controller.hpp
 * @author    lupinjia(jason_1120202397@163.com)
 * @brief     This file is where you define your own controller with specific logic
 *            ! Please create and modify your own inheritance of BasicController, do not modify BasicController!
 * @version   1.0
 * @date      2024-12-09
 * 
 * @copyright HYGroup @ BIT 2024
 * 
 */
#ifndef __USER_CONTROLLER_HPP__
#define __USER_CONTROLLER_HPP__

#include <array>
#include <vector>
#include <filesystem>
#include <fstream>
#include <string>
#include <deque>

#ifdef INFER_LIBTORCH
    #include "torch/script.h"
#endif
#ifdef INFER_OPENVINO
    #include <openvino/openvino.hpp>
#endif
#include "robot_interface.hpp"
#include "cfg.hpp"
#include "common/mathTools.h"
#include <spdlog/spdlog.h>
#include "gamepad.hpp"

using namespace legged::common;
namespace fs = std::filesystem;

class BasicUserController
{
public:
    BasicUserController() {}

    virtual void loadParam() = 0;

    virtual void loadPolicy() = 0;

    virtual void reset(BasicRobotInterface &robot_interface) = 0;

    virtual void getInput(BasicRobotInterface &robot_interface, Gamepad &gamepad) = 0;

    virtual void dummyCalculate() = 0;

    virtual void calculate() = 0;

    virtual std::vector<float> getLog() = 0;

    /**
    * @brief 保存初始位置
    * @param robot_interface
    * @note  在阻尼状态时调用, 读取关节位置保存为初始位置
    */
    void save_jpos(BasicRobotInterface &robot_interface)
    {
        std::copy(robot_interface.jpos.begin(), robot_interface.jpos.end(), start_pos.begin());
    }

    float dt;
    float stand_kp;
    float stand_kd;
    float ctrl_kp;
    float ctrl_kd;
    float rpy_offset_increment;
    std::array<float, 6> start_pos; // initial pos when resting
    std::array<float, 6> stand_pos; // final pos for standing
    std::array<float, 6> jpos_des;  // calculated desired joint pos
    std::array<float, 6> lower_limit;
    std::array<float, 6> upper_limit;
};


// UserController为实际进行控制所需计算的类
class EEController : public BasicUserController
{
public:
    EEController(const std::string& cfg_file)
    {
        heading = 0.0;
        // reset params
        cmd.fill(0.0);
        projected_gravity.fill(0.);
        projected_gravity.at(2) = -1.0;
        base_ang_vel.fill(0.0);
        rpy.fill(0.0);
        jpos_processed.fill(0.0);
        jvel.fill(0.0);
        actions.fill(0.0);
        clock_input.fill(0.0);
        gait_time = 0.0;
        last_actions.fill(0.0);
        config_file_name = cfg_file;

        #ifdef INFER_OPENVINO
        // -------- Get OpenVINO runtime version --------
        spdlog::info("{} : {}", ov::get_openvino_version().description, ov::get_openvino_version().buildNumber);
        // // -------- Step 1. Initialize OpenVINO Runtime Core --------
        // ov::Core core;
        // -------- Step 2. Get list of available devices --------
        std::vector<std::string> availableDevices = core.get_available_devices();
        // -------- Step 3. Query and print supported metrics and config keys --------
        spdlog::info("Available devices: ");
        for (auto&& device : availableDevices) {
            spdlog::info("{}", device);
        }
        #endif

    }

    /**
     * @brief 从json文件中读取参数
     * @param param_folder 参数文件夹路径
     * @note  读取参数文件rl_params.json, 包含以下参数:
     * @note  - dt: 控制周期
    */
    void loadParam()
    {
        // load param file
        EECfg cfg(config_file_name);

        // get data from json
        dt = cfg.dt;
        policy_name = cfg.policy_name;
        stand_kp = cfg.stand_kp;
        stand_kd = cfg.stand_kd;
        action_scale = cfg.action_scale;
        lin_vel_scale = cfg.lin_vel_scale;
        ang_vel_scale = cfg.ang_vel_scale;
        dof_pos_scale = cfg.dof_pos_scale;
        dof_vel_scale = cfg.dof_vel_scale;
        ctrl_kp = cfg.ctrl_kp;
        ctrl_kd = cfg.ctrl_kd;
        frame_stack = cfg.frame_stack;
        num_single_obs = cfg.num_single_obs;
        cmd_lin_x_scale = cfg.cmd_lin_x_scale;
        cmd_lin_y_scale = cfg.cmd_lin_y_scale;
        cmd_ang_z_scale = cfg.cmd_ang_z_scale;
        min_lin_x = cfg.min_lin_x;
        max_lin_x = cfg.max_lin_x;
        gait_period = cfg.gait_period;
        theta_left = cfg.theta_left;
        theta_right = cfg.theta_right;
        rpy_offset_increment = cfg.rpy_offset_increment;
        for (int i = 0; i < 6; ++i)
        {
            stand_pos.at(i) = cfg.stand_pos.at(i);
            start_pos.at(i) = cfg.start_pos.at(i);
            lower_limit.at(i) = cfg.lower_limit.at(i);
            upper_limit.at(i) = cfg.upper_limit.at(i);
        }
        // prepare history buffers
        single_step_obs.resize(num_single_obs);
        single_step_obs.clear();
        for(int i = 0; i < num_single_obs; ++i)
        {
            single_step_obs.push_back(0.0);
        }
        history_obs.resize(frame_stack);
        for(int i = 0; i < frame_stack; ++i)
        {
            history_obs.at(i).resize(num_single_obs);
            history_obs.at(i) = single_step_obs;
        }
    }
    /**
     * @brief 载入运动策略
     * @param model_name 模型名称
     * @note  载入后缀为.pt的模型文件, 该模型文件使用torch.jit.save()保存.
     * @note  默认模型文件路径为../models/
    */
    void loadPolicy()
    {
        fs::path model_path = fs::current_path() / "../models";
        std::string policy_file = model_path / policy_name;
        #ifdef INFER_LIBTORCH
        policy = torch::jit::load(policy_file);
        #endif
        #ifdef INFER_OPENVINO
        policy = core.read_model(policy_file);
        ov::CompiledModel compiled_model = core.compile_model(policy, "CPU");
        infer_request = compiled_model.create_infer_request();
        #endif
        spdlog::info("Load policy from: {}", policy_file);
    }

    void getInput(BasicRobotInterface &robot_interface, Gamepad &gamepad)
    {
        // record current observation
        std::copy(robot_interface.gyro.begin(), robot_interface.gyro.end(), base_ang_vel.begin());
        std::copy(robot_interface.projected_gravity.begin(), robot_interface.projected_gravity.end(), projected_gravity.begin());
        std::copy(robot_interface.rpy.begin(), robot_interface.rpy.end(), rpy.begin());

        // cmd
        cmd.at(0) = gamepad.ly * cmd_lin_x_scale + gamepad.ry * cmd_lin_x_scale;
        cmd.at(0) = clip(cmd.at(0), min_lin_x, max_lin_x);
        cmd.at(1) = -gamepad.lx * cmd_lin_y_scale;
        cmd.at(2) = -gamepad.rx * cmd_ang_z_scale;
        // record robot state
        for(uint8_t i = 0; i < 6; i++)
        {
            jpos_processed.at(i) = robot_interface.jpos.at(i) - stand_pos.at(i);
            jvel.at(i) = robot_interface.jvel.at(i);
        }
    }

    void reset(BasicRobotInterface &robot_interface)
    {
        // prepare history buffers
        single_step_obs.resize(num_single_obs);
        single_step_obs.clear();
        for(int i = 0; i < num_single_obs; ++i)
        {
            single_step_obs.push_back(0.0);
        }
        history_obs.resize(frame_stack);
        for(int i = 0; i < frame_stack; ++i)
        {
            history_obs.at(i).resize(num_single_obs);
            history_obs.at(i) = single_step_obs;
        }
        jpos_des = stand_pos;
        // reset params
        cmd.fill(0.0);
        projected_gravity.fill(0.);
        projected_gravity.at(2) = -1.0;
        base_ang_vel.fill(0.0);
        heading = 0.0;
        rpy.fill(0.0);
        jpos_processed.fill(0.0);
        jvel.fill(0.0);
        actions.fill(0.0);
        last_actions.fill(0.0);
        clock_input.fill(0.0);
        gait_time = 0.0;
    }

    void dummyCalculate()
    {
        calc_clock_input();
        for(int i = 0; i < num_single_obs; ++i)
        {
            single_step_obs.at(i) = 0.0;
        }
        history_obs.pop_front();
        history_obs.push_back(single_step_obs);
        #ifdef INFER_LIBTORCH
        std::vector<torch::jit::IValue> policy_input;
        torch::Tensor policy_input_tensor = torch::zeros({1, frame_stack*num_single_obs}); // 输入的观测长度为58
        for(int i = 0; i < frame_stack; ++i)
        {
            for(int j = 0; j < num_single_obs; ++j)
            {
                policy_input_tensor[0][i*num_single_obs + j] = history_obs.at(i).at(j);
            }
        }
        policy_input.push_back(policy_input_tensor);
        torch::Tensor policy_output_tensor = policy.forward(policy_input).toTensor();
        #endif
        #ifdef INFER_OPENVINO
        ov::element::Type ov_type = ov::element::f32;
        ov::Shape ov_shape = {1, frame_stack * num_single_obs};
        ov::Tensor policy_input_tensor(ov_type, ov_shape);
        for(int i = 0; i < frame_stack; ++i)
        {
            for(int j = 0; j < num_single_obs; ++j)
            {
                policy_input_tensor.data<float>()[i * num_single_obs + j] = history_obs.at(i).at(j);
            }
        }
        infer_request.set_input_tensor(policy_input_tensor);
        infer_request.infer();
        ov::Tensor output_tensor = infer_request.get_output_tensor();
        #endif
    }

    void calculate()
    {
        calc_clock_input();
        // put current observation into history buffer
        fill_single_step_obs();
        history_obs.pop_front();
        history_obs.push_back(single_step_obs);
        #ifdef INFER_LIBTORCH
        std::vector<torch::jit::IValue> policy_input;
        torch::Tensor policy_input_tensor = torch::zeros({1, frame_stack*num_single_obs});
        for(int i = 0; i < frame_stack; ++i)
        {
            for(int j = 0; j < num_single_obs; ++j)
            {
                policy_input_tensor[0][i*num_single_obs + j] = history_obs.at(i).at(j);
            }
        }
        policy_input.push_back(policy_input_tensor);
        torch::Tensor policy_output_tensor = policy.forward(policy_input).toTensor();
        /***** 计算期望位置 *****/
        std::array<float, 6> actions_scaled;
        for(int i = 0; i < 6; ++i)// 将输出的动作保存下来
        {
            last_actions.at(i) = actions.at(i); // 保存上一次的动作
            actions.at(i) = policy_output_tensor[0][i].item<float>();
            actions_scaled.at(i) = actions.at(i) * action_scale;
            jpos_des.at(i) = stand_pos.at(i) + actions_scaled.at(i);
        }
        #endif
        #ifdef INFER_OPENVINO
        ov::element::Type ov_type = ov::element::f32;
        ov::Shape ov_shape = {1, frame_stack * num_single_obs};
        ov::Tensor policy_input_tensor(ov_type, ov_shape);
        for(int i = 0; i < frame_stack; ++i)
        {
            for(int j = 0; j < num_single_obs; ++j)
            {
                    policy_input_tensor.data<float>()[i * num_single_obs + j] = history_obs.at(i).at(j);
            }
        }
        infer_request.set_input_tensor(policy_input_tensor);
        infer_request.infer();
        ov::Tensor output_tensor = infer_request.get_output_tensor();
        /***** 计算期望位置 *****/
        std::array<float, 6> actions_scaled;
        for(int i = 0; i < 6; ++i)// 将输出的动作保存下来
        {
            last_actions.at(i) = actions.at(i); // 保存上一次的动作
            actions.at(i) = output_tensor.data<float>()[i]; // 输出动作
            actions_scaled.at(i) = actions.at(i) * action_scale;
            jpos_des.at(i) = stand_pos.at(i) + actions_scaled.at(i);
        }
        #endif
    }

    std::vector<float> getLog()
    {
        // record input, output and other info into a vector
        std::vector<float> log;
        for(uint8_t i = 0; i < 3; i++) // cmd
        {
          log.push_back(cmd.at(i));
        }
        for(uint8_t i = 0; i < 3; i++) // projected_gravity
        {
          log.push_back(projected_gravity.at(i));
        }
        for(uint8_t i = 0; i < 3; i++) // base_ang_vel
        {
          log.push_back(base_ang_vel.at(i));
        }
        for(uint8_t i = 0; i < 6; i++) // dof_pos
        {
            log.push_back(jpos_processed.at(i) + stand_pos.at(i));
        }
        for(uint8_t i = 0; i < 6; i++) // dof_vel
        {
            log.push_back(jvel.at(i));
        }
        for(uint8_t i = 0; i < 4; i++) // clock_input
        {
            log.push_back(clock_input.at(i));
        }
        log.push_back(gait_time);
        log.push_back(gait_time / gait_period);
        // for(uint8_t i = 0; i < 6; i++) // last jpos_des, leading to current jpos
        // {
        //   log.push_back(last_actions.at(i) * action_scale + stand_pos.at(i));
        // }
        return log;
    }

    std::string config_file_name;
    // control params
    float heading;
    std::array<float, 3> rpy;
    std::string policy_name;
    float gait_period;
    float theta_left;
    float theta_right;
    float gait_time;

    // observation
    std::array<float, 3> cmd;
    std::array<float, 3> projected_gravity;
    std::array<float, 3> base_ang_vel;
    std::array<float, 6> jpos_processed;
    std::array<float, 6> jvel;
    std::array<float, 6> actions;
    std::array<float, 4> clock_input;
    std::array<float, 6> last_actions; // 上一时刻的动作
    std::vector<float> single_step_obs; // 用于记录单步观测数据
    std::deque<std::vector<float>> history_obs; // 用于记录历史观测数据

    // normalization params
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float action_scale;

    float cmd_lin_x_scale;
    float cmd_lin_y_scale;
    float cmd_ang_z_scale;
    float min_lin_x;
    float max_lin_x;

    // NN model
    #ifdef INFER_LIBTORCH
    torch::jit::script::Module policy;
    #endif
    #ifdef INFER_OPENVINO
    std::shared_ptr<ov::Model> policy;
    ov::Core core;
    ov::InferRequest infer_request;
    #endif
    int frame_stack;
    int num_single_obs; // length of single step observation
private:

    void calc_clock_input()
    {
        gait_time += dt;
        if(gait_time > (gait_period - (dt / 2.0)))
        {
            gait_time = 0.0;
        }
        float phi = gait_time / gait_period; // phase [0, 1)
        clock_input.at(0) = sin(2.0 * M_PI * (phi + theta_left));
        clock_input.at(1) = sin(2.0 * M_PI * (phi + theta_right));
        clock_input.at(2) = cos(2.0 * M_PI * (phi + theta_left));
        clock_input.at(3) = cos(2.0 * M_PI * (phi + theta_right));
    }

    void fill_single_step_obs()
    {
        single_step_obs.at(0) = cmd.at(0) * lin_vel_scale;
        single_step_obs.at(1) = cmd.at(1) * lin_vel_scale;
        single_step_obs.at(2) = cmd.at(2) * ang_vel_scale;
        for(int i = 0; i < 3; ++i)
        {
            single_step_obs.at(i+3) = projected_gravity.at(i);
            single_step_obs.at(i+6) = base_ang_vel.at(i) * ang_vel_scale;
        }
        for(int i = 0; i < 6; ++i)
        {
            single_step_obs.at(i+9) = jpos_processed.at(i) * dof_pos_scale;
            single_step_obs.at(i+15) = jvel.at(i) * dof_vel_scale;
            single_step_obs.at(i+21) = actions.at(i);
        }
        for(int i = 0; i < 4; ++i)
        {
            single_step_obs.at(i+27) = clock_input.at(i);
        }
    }

};

// Demo controller that uses depth image as part of the observation
// The outcome performance on agilepf is not very good, this controller is preserved
class DepthController : public BasicUserController
{
public:
    DepthController(const std::string& cfg_file)
    {
        heading = 0.0;
        // reset params
        cmd.fill(0.0);
        projected_gravity.fill(0.);
        projected_gravity.at(2) = -1.0;
        base_ang_vel.fill(0.0);
        rpy.fill(0.0);
        jpos_processed.fill(0.0);
        jvel.fill(0.0);
        actions.fill(0.0);
        clock_input.fill(0.0);
        gait_time = 0.0;
        last_actions.fill(0.0);
        config_file_name = cfg_file;
    }

    /**
     * @brief 从json文件中读取参数
     * @param param_folder 参数文件夹路径
     * @note  读取参数文件rl_params.json, 包含以下参数:
     * @note  - dt: 控制周期
    */
    void loadParam()
    {
        // load param file
        DepthCfg cfg(config_file_name);

        // get data from json
        dt = cfg.dt;
        policy_name = cfg.policy_name;
        stand_kp = cfg.stand_kp;
        stand_kd = cfg.stand_kd;
        action_scale = cfg.action_scale;
        lin_vel_scale = cfg.lin_vel_scale;
        ang_vel_scale = cfg.ang_vel_scale;
        dof_pos_scale = cfg.dof_pos_scale;
        dof_vel_scale = cfg.dof_vel_scale;
        ctrl_kp = cfg.ctrl_kp;
        ctrl_kd = cfg.ctrl_kd;
        num_single_obs = cfg.num_single_obs;
        cmd_lin_x_scale = cfg.cmd_lin_x_scale;
        cmd_lin_y_scale = cfg.cmd_lin_y_scale;
        cmd_ang_z_scale = cfg.cmd_ang_z_scale;
        min_lin_x = cfg.min_lin_x;
        max_lin_x = cfg.max_lin_x;
        gait_period = cfg.gait_period;
        theta_left = cfg.theta_left;
        theta_right = cfg.theta_right;
        depth_image_width = cfg.depth_image_width;
        depth_image_height = cfg.depth_image_height;
        depth_image_scale = cfg.depth_image_scale;
        depth_image_obs.resize(depth_image_width * depth_image_height);
        for (int i = 0; i < 6; ++i)
        {
            stand_pos.at(i) = cfg.stand_pos.at(i);
            start_pos.at(i) = cfg.start_pos.at(i);
            lower_limit.at(i) = cfg.lower_limit.at(i);
            upper_limit.at(i) = cfg.upper_limit.at(i);
        }
        // prepare history buffers
        single_step_obs.resize(num_single_obs);
        single_step_obs.clear();
        for(int i = 0; i < num_single_obs; ++i)
        {
            single_step_obs.push_back(0.0);
        }
    }
    /**
     * @brief 载入运动策略
     * @param model_name 模型名称
     * @note  载入后缀为.pt的模型文件, 该模型文件使用torch.jit.save()保存.
     * @note  默认模型文件路径为../models/
    */
    void loadPolicy()
    {
        fs::path model_path = fs::current_path() / "../models";
        std::string policy_file = model_path / policy_name;
        #ifdef INFER_LIBTORCH
        policy = torch::jit::load(policy_file);
        #endif
        spdlog::info("Load policy from: {}", policy_file);
    }

    void getInput(BasicRobotInterface &robot_interface, Gamepad &gamepad)
    {
        // record current observation
        std::copy(robot_interface.gyro.begin(), robot_interface.gyro.end(), base_ang_vel.begin());
        std::copy(robot_interface.projected_gravity.begin(), robot_interface.projected_gravity.end(), projected_gravity.begin());
        std::copy(robot_interface.rpy.begin(), robot_interface.rpy.end(), rpy.begin());

        // cmd
        cmd.at(0) = gamepad.ly * cmd_lin_x_scale + gamepad.ry * cmd_lin_x_scale;
        cmd.at(0) = clip(cmd.at(0), min_lin_x, max_lin_x);
        cmd.at(1) = -gamepad.lx * cmd_lin_y_scale;
        cmd.at(2) = -gamepad.rx * cmd_ang_z_scale;
        // record robot state
        for(uint8_t i = 0; i < 6; i++)
        {
            jpos_processed.at(i) = robot_interface.jpos.at(i) - stand_pos.at(i);
            jvel.at(i) = robot_interface.jvel.at(i);
        }

        // record depth image
        if(depth_image_obs.size() != robot_interface.depth_image_width * robot_interface.depth_image_height)
        {
            depth_image_obs.resize(robot_interface.depth_image_width * robot_interface.depth_image_height);
        }
        std::copy(robot_interface.depth_image_buffer.begin(), robot_interface.depth_image_buffer.end(), depth_image_obs.begin());
        
    }

    void reset(BasicRobotInterface &robot_interface)
    {
        // prepare history buffers
        single_step_obs.resize(num_single_obs);
        single_step_obs.clear();
        for(int i = 0; i < num_single_obs; ++i)
        {
            single_step_obs.push_back(0.0);
        }
        jpos_des = stand_pos;
        // reset params
        cmd.fill(0.0);
        projected_gravity.fill(0.);
        projected_gravity.at(2) = -1.0;
        base_ang_vel.fill(0.0);
        heading = 0.0;
        rpy.fill(0.0);
        jpos_processed.fill(0.0);
        jvel.fill(0.0);
        actions.fill(0.0);
        last_actions.fill(0.0);
        clock_input.fill(0.0);
        gait_time = 0.0;
    }

    void dummyCalculate()
    {
        calc_clock_input();
        for(int i = 0; i < num_single_obs; ++i)
        {
            single_step_obs.at(i) = 0.0;
        }
        #ifdef INFER_LIBTORCH
        std::vector<torch::jit::IValue> policy_input;
        torch::Tensor cur_obs_tensor = torch::zeros({1, num_single_obs});
        for(int j = 0; j < num_single_obs; ++j)
        {
            cur_obs_tensor[0][j] = single_step_obs.at(j);
        }
        torch::Tensor depth_image_tensor = torch::zeros({1, 1, depth_image_height, depth_image_width});
        for(int h = 0; h < depth_image_height; ++h)
        {
            for(int w = 0; w < depth_image_width; ++w)
            {
                int idx = h * depth_image_width + w;
                depth_image_tensor[0][0][h][w] = (depth_image_obs.at(idx) - 0.5) * depth_image_scale;
            }
        }
        policy_input.push_back(cur_obs_tensor);
        policy_input.push_back(depth_image_tensor);
        torch::Tensor policy_output_tensor = policy.forward(policy_input).toTensor();
        #endif
    }

    void calculate()
    {
        calc_clock_input();
        // put current observation into history buffer
        fill_single_step_obs();
        #ifdef INFER_LIBTORCH
        std::vector<torch::jit::IValue> policy_input;
        torch::Tensor cur_obs_tensor = torch::zeros({1, num_single_obs});
        for(int j = 0; j < num_single_obs; ++j)
        {
            cur_obs_tensor[0][j] = single_step_obs.at(j);
        }
        torch::Tensor depth_image_tensor = torch::zeros({1, 1, depth_image_height, depth_image_width});
        for(int h = 0; h < depth_image_height; ++h)
        {
            for(int w = 0; w < depth_image_width; ++w)
            {
                int idx = h * depth_image_width + w;
                depth_image_tensor[0][0][h][w] = (depth_image_obs.at(idx) - 0.5) * depth_image_scale;
            }
        }
        policy_input.push_back(cur_obs_tensor);
        policy_input.push_back(depth_image_tensor);
        torch::Tensor policy_output_tensor = policy.forward(policy_input).toTensor();
        /***** 计算期望位置 *****/
        std::array<float, 6> actions_scaled;
        for(int i = 0; i < 6; ++i)// 将输出的动作保存下来
        {
            last_actions.at(i) = actions.at(i); // 保存上一次的动作
            actions.at(i) = policy_output_tensor[0][i].item<float>();
            actions_scaled.at(i) = actions.at(i) * action_scale;
            jpos_des.at(i) = stand_pos.at(i) + actions_scaled.at(i);
        }
        #endif
    }

    std::vector<float> getLog()
    {
        // record input, output and other info into a vector
        std::vector<float> log;
        for(uint8_t i = 0; i < 3; i++) // cmd
        {
          log.push_back(cmd.at(i));
        }
        for(uint8_t i = 0; i < 3; i++) // projected_gravity
        {
          log.push_back(projected_gravity.at(i));
        }
        for(uint8_t i = 0; i < 3; i++) // base_ang_vel
        {
          log.push_back(base_ang_vel.at(i));
        }
        for(uint8_t i = 0; i < 6; i++) // dof_pos
        {
            log.push_back(jpos_processed.at(i) + stand_pos.at(i));
        }
        for(uint8_t i = 0; i < 6; i++) // dof_vel
        {
            log.push_back(jvel.at(i));
        }
        for(uint8_t i = 0; i < 4; i++) // clock_input
        {
            log.push_back(clock_input.at(i));
        }
        log.push_back(gait_time);
        log.push_back(gait_time / gait_period);
        // for(uint8_t i = 0; i < 6; i++) // last jpos_des, leading to current jpos
        // {
        //   log.push_back(last_actions.at(i) * action_scale + stand_pos.at(i));
        // }
        return log;
    }

    std::string config_file_name;
    // control params
    float heading;
    std::array<float, 3> rpy;
    std::string policy_name;
    float gait_period;
    float theta_left;
    float theta_right;
    float gait_time;

    // observation
    std::array<float, 3> cmd;
    std::array<float, 3> projected_gravity;
    std::array<float, 3> base_ang_vel;
    std::array<float, 6> jpos_processed;
    std::array<float, 6> jvel;
    std::array<float, 6> actions;
    std::array<float, 4> clock_input;
    std::array<float, 6> last_actions; // 上一时刻的动作
    std::vector<float> single_step_obs; // 用于记录单步观测数据
    std::vector<float> depth_image_obs; // 用于记录深度图像观测数据

    // normalization params
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float action_scale;
    float depth_image_scale;

    // depth image params
    int depth_image_width;
    int depth_image_height;

    float cmd_lin_x_scale;
    float cmd_lin_y_scale;
    float cmd_ang_z_scale;
    float min_lin_x;
    float max_lin_x;

    // NN model
    #ifdef INFER_LIBTORCH
    torch::jit::script::Module policy;
    #endif
    int num_single_obs; // length of single step observation
private:

    void calc_clock_input()
    {
        gait_time += dt;
        if(gait_time > (gait_period - (dt / 2.0)))
        {
            gait_time = 0.0;
        }
        float phi = gait_time / gait_period; // phase [0, 1)
        clock_input.at(0) = sin(2.0 * M_PI * (phi + theta_left));
        clock_input.at(1) = sin(2.0 * M_PI * (phi + theta_right));
        clock_input.at(2) = cos(2.0 * M_PI * (phi + theta_left));
        clock_input.at(3) = cos(2.0 * M_PI * (phi + theta_right));
    }

    void fill_single_step_obs()
    {
        single_step_obs.at(0) = cmd.at(0) * lin_vel_scale;
        single_step_obs.at(1) = cmd.at(1) * lin_vel_scale;
        single_step_obs.at(2) = cmd.at(2) * ang_vel_scale;
        for(int i = 0; i < 3; ++i)
        {
            single_step_obs.at(i+3) = projected_gravity.at(i);
            single_step_obs.at(i+6) = base_ang_vel.at(i) * ang_vel_scale;
        }
        for(int i = 0; i < 6; ++i)
        {
            single_step_obs.at(i+9) = jpos_processed.at(i) * dof_pos_scale;
            single_step_obs.at(i+15) = jvel.at(i) * dof_vel_scale;
            single_step_obs.at(i+21) = actions.at(i);
        }
        for(int i = 0; i < 4; ++i)
        {
            single_step_obs.at(i+27) = clock_input.at(i);
        }
    }


};

#endif