#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

struct PolicyCfg
{
    explicit PolicyCfg(const std::string& filename)
    {
        const auto yaml = YAML::LoadFile(filename);
        policy_name = yaml["policy_name"].as<std::string>();
        dt = yaml["dt"].as<float>();
        stand_kp = yaml["stand_kp"].as<float>();
        stand_kd = yaml["stand_kd"].as<float>();
        cmd_lin_x_scale = yaml["cmd_lin_x_scale"].as<float>();
        cmd_lin_y_scale = yaml["cmd_lin_y_scale"].as<float>();
        cmd_ang_z_scale = yaml["cmd_ang_z_scale"].as<float>();
        min_lin_x = yaml["min_lin_x"].as<float>();
        max_lin_x = yaml["max_lin_x"].as<float>();
        rpy_offset_increment = yaml["rpy_offset_increment"].as<float>();
        readArray(yaml["start_pos"], start_pos, "start_pos");
        readArray(yaml["lower_limit"], lower_limit, "lower_limit");
        readArray(yaml["upper_limit"], upper_limit, "upper_limit");
        if (yaml["default_joint_pos_offset"]) {
            try {
                readArray(yaml["default_joint_pos_offset"], default_joint_pos_offset, "default_joint_pos_offset");
            } catch (const std::exception& error) {
                throw std::runtime_error("Invalid default_joint_pos_offset in policy YAML: " + std::string(error.what()));
            }
        }
        if (policy_name.empty() || !std::isfinite(dt) || dt <= 0 ||
            !std::isfinite(stand_kp) || !std::isfinite(stand_kd) ||
            !std::isfinite(cmd_lin_x_scale) || !std::isfinite(cmd_lin_y_scale) ||
            !std::isfinite(cmd_ang_z_scale) || !std::isfinite(min_lin_x) ||
            !std::isfinite(max_lin_x) || min_lin_x > max_lin_x ||
            !std::isfinite(rpy_offset_increment))
            throw std::runtime_error("Invalid finite/range value in policy YAML: " + filename);
        for (size_t i = 0; i < 6; ++i)
            if (lower_limit[i] > upper_limit[i])
                throw std::runtime_error("Invalid joint limits in policy YAML");
    }

    std::string policy_name;
    float dt{}, stand_kp{}, stand_kd{};
    float cmd_lin_x_scale{}, cmd_lin_y_scale{}, cmd_ang_z_scale{};
    float min_lin_x{}, max_lin_x{}, rpy_offset_increment{};
    std::array<float, 6> start_pos{}, lower_limit{}, upper_limit{}, default_joint_pos_offset{};

private:
    static void readArray(const YAML::Node& node, std::array<float, 6>& out, const char* name)
    {
        if (!node || !node.IsSequence() || node.size() != out.size())
            throw std::runtime_error(std::string("Expected six YAML values for ") + name);
        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = node[i].as<float>();
            if (!std::isfinite(out[i]))
                throw std::runtime_error(std::string("Non-finite YAML value for ") + name);
        }
    }
};
