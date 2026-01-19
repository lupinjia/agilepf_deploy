#pragma once

#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>

class BaseCfg
{
public:
    BaseCfg(const std::string &filename){};

};

class EECfg : public BaseCfg
{
public:
    EECfg(const std::string &filename) : BaseCfg(filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            policy_name = cfg["policy_name"].as<std::string>();
            dt = cfg["dt"].as<float>();
            stand_kp = cfg["stand_kp"].as<float>();
            stand_kd = cfg["stand_kd"].as<float>();
            ctrl_kp = cfg["ctrl_kp"].as<float>();
            ctrl_kd = cfg["ctrl_kd"].as<float>();
            action_scale = cfg["action_scale"].as<float>();
            lin_vel_scale = cfg["lin_vel_scale"].as<float>();
            ang_vel_scale = cfg["ang_vel_scale"].as<float>();
            dof_pos_scale = cfg["dof_pos_scale"].as<float>();
            dof_vel_scale = cfg["dof_vel_scale"].as<float>();
            cmd_lin_x_scale = cfg["cmd_lin_x_scale"].as<float>();
            cmd_lin_y_scale = cfg["cmd_lin_y_scale"].as<float>();
            cmd_ang_z_scale = cfg["cmd_ang_z_scale"].as<float>();
            min_lin_x = cfg["min_lin_x"].as<float>();
            max_lin_x = cfg["max_lin_x"].as<float>();
            frame_stack = cfg["frame_stack"].as<int>();
            num_single_obs = cfg["num_single_obs"].as<int>();
            gait_period = cfg["gait_period"].as<float>();
            theta_left = cfg["theta_left"].as<float>();
            theta_right = cfg["theta_right"].as<float>();
            for(const auto& v : cfg["stand_pos"])
            {
                stand_pos.push_back(v.as<float>());
            }
            for(const auto& v : cfg["start_pos"])
            {
                start_pos.push_back(v.as<float>());
            }
            for(const auto& v : cfg["lower_limit"])
            {
                lower_limit.push_back(v.as<float>());
            }
            for(const auto& v : cfg["upper_limit"])
            {
                upper_limit.push_back(v.as<float>());
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
    
    std::string policy_name;
    float stand_kp;
    float stand_kd;
    float dt;
    float action_scale;
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float ctrl_kp;
    float ctrl_kd;
    float cmd_lin_x_scale;
    float cmd_lin_y_scale;
    float cmd_ang_z_scale;
    float min_lin_x;
    float max_lin_x;
    float gait_period;
    float theta_left;
    float theta_right;
    int frame_stack; // how many frames to stack for observation
    int num_single_obs; // length of single step observation
        
    std::vector<float> stand_pos;
    std::vector<float> start_pos;
    std::vector<float> lower_limit;
    std::vector<float> upper_limit;
        
};

class DepthCfg : public BaseCfg
{
public:
    DepthCfg(const std::string &filename) : BaseCfg(filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            policy_name = cfg["policy_name"].as<std::string>();
            dt = cfg["dt"].as<float>();
            stand_kp = cfg["stand_kp"].as<float>();
            stand_kd = cfg["stand_kd"].as<float>();
            ctrl_kp = cfg["ctrl_kp"].as<float>();
            ctrl_kd = cfg["ctrl_kd"].as<float>();
            action_scale = cfg["action_scale"].as<float>();
            lin_vel_scale = cfg["lin_vel_scale"].as<float>();
            ang_vel_scale = cfg["ang_vel_scale"].as<float>();
            dof_pos_scale = cfg["dof_pos_scale"].as<float>();
            dof_vel_scale = cfg["dof_vel_scale"].as<float>();
            cmd_lin_x_scale = cfg["cmd_lin_x_scale"].as<float>();
            cmd_lin_y_scale = cfg["cmd_lin_y_scale"].as<float>();
            cmd_ang_z_scale = cfg["cmd_ang_z_scale"].as<float>();
            min_lin_x = cfg["min_lin_x"].as<float>();
            max_lin_x = cfg["max_lin_x"].as<float>();
            num_single_obs = cfg["num_single_obs"].as<int>();
            gait_period = cfg["gait_period"].as<float>();
            theta_left = cfg["theta_left"].as<float>();
            theta_right = cfg["theta_right"].as<float>();
            depth_image_width = cfg["depth_image_width"].as<int>();
            depth_image_height = cfg["depth_image_height"].as<int>();
            depth_image_scale = cfg["depth_image_scale"].as<float>();
            for(const auto& v : cfg["stand_pos"])
            {
                stand_pos.push_back(v.as<float>());
            }
            for(const auto& v : cfg["start_pos"])
            {
                start_pos.push_back(v.as<float>());
            }
            for(const auto& v : cfg["lower_limit"])
            {
                lower_limit.push_back(v.as<float>());
            }
            for(const auto& v : cfg["upper_limit"])
            {
                upper_limit.push_back(v.as<float>());
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
    
    std::string policy_name;
    float stand_kp;
    float stand_kd;
    float dt;
    float action_scale;
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float ctrl_kp;
    float ctrl_kd;
    float cmd_lin_x_scale;
    float cmd_lin_y_scale;
    float cmd_ang_z_scale;
    float min_lin_x;
    float max_lin_x;
    float gait_period;
    float theta_left;
    float theta_right;
    int num_single_obs; // length of single step observation
    int depth_image_width;
    int depth_image_height;
    float depth_image_scale;

    std::vector<float> stand_pos;
    std::vector<float> start_pos;
    std::vector<float> lower_limit;
    std::vector<float> upper_limit;
        
};