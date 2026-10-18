#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "user_controller.hpp"

namespace {
constexpr size_t kTermCount = 6;
constexpr size_t kHistory = 3;
constexpr size_t kInputSize = 81;
using Frame = std::array<std::array<float, 6>, kTermCount>;
using Observation = std::array<float, kInputSize>;
using Actions = std::array<float, 6>;
constexpr Actions kActionScale = {0.749f, 0.749f, 0.749f, 0.749f, 0.749f, 0.749f};
constexpr Actions kDefaultPose = {0.0f, 0.5f, -0.5f, 0.0f, 0.5f, -0.5f};

struct TemporaryDirectory {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("agilepf_default_offset_smoke_" + std::to_string(
            std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    TemporaryDirectory() { std::filesystem::create_directories(path); }
    ~TemporaryDirectory() { std::error_code ignored; std::filesystem::remove_all(path, ignored); }
};

void writePolicyConfig(const std::filesystem::path& path, const std::string& policy_name,
                       const std::string& offset_line = {})
{
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot create temporary policy config");
    out << "policy_name: \"" << policy_name << "\"\n"
        << "dt: 0.02\nstand_kp: 60.0\nstand_kd: 3.0\n"
        << "cmd_lin_x_scale: 1.0\ncmd_lin_y_scale: 0.4\ncmd_ang_z_scale: 1.0\n"
        << "min_lin_x: -0.5\nmax_lin_x: 1.0\nrpy_offset_increment: 0.01\n"
        << "start_pos: [0.32, 1.21, -1.98, -0.32, 1.21, -1.98]\n"
        << "lower_limit: [-0.4, -1.1, -1.8, -0.4, -1.1, -1.8]\n"
        << "upper_limit: [0.4, 2.7, -0.1, 0.4, 2.7, -0.1]\n";
    if (!offset_line.empty()) out << offset_line << '\n';
    if (!out) throw std::runtime_error("failed writing temporary policy config");
}

void requireOffsetConfigRejected(const std::filesystem::path& path, const std::string& policy_name,
                                 const std::string& offset_line, const char* expected_error)
{
    writePolicyConfig(path, policy_name, offset_line);
    try {
        (void)PolicyCfg(path.string());
    } catch (const std::exception& error) {
        if (std::string(error.what()).find(expected_error) != std::string::npos) return;
        throw std::runtime_error(std::string("unexpected offset validation error: ") + error.what());
    }
    throw std::runtime_error("malformed default_joint_pos_offset was accepted");
}

void requireClose(float actual, float expected, const char* label)
{
    if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-5f)
        throw std::runtime_error(std::string(label) + " differs from direct ONNX reference");
}

void requireArrayClose(const std::array<float, 6>& actual, const std::array<float, 6>& expected,
                       const char* label)
{
    for (size_t i = 0; i < actual.size(); ++i)
        requireClose(actual[i], expected[i], label);
}

Frame makeExpectedFrame(const BasicRobotInterface& robot, const Gamepad& pad,
                        const Actions& previous_actions, const Actions& stand_pos)
{
    Frame f{};
    const std::array<std::array<float, 6>, kTermCount> raw = {{
        {{pad.ly + pad.ry, -pad.lx * 0.4f, -pad.rx, 0, 0, 0}},
        {{robot.gyro[0], robot.gyro[1], robot.gyro[2], 0, 0, 0}},
        {{robot.projected_gravity[0], robot.projected_gravity[1], robot.projected_gravity[2], 0, 0, 0}},
        {{robot.jpos[0] - stand_pos[0], robot.jpos[1] - stand_pos[1], robot.jpos[2] - stand_pos[2],
          robot.jpos[3] - stand_pos[3], robot.jpos[4] - stand_pos[4], robot.jpos[5] - stand_pos[5]}},
        {{robot.jvel[0], robot.jvel[1], robot.jvel[2], robot.jvel[3], robot.jvel[4], robot.jvel[5]}},
        {{previous_actions[0], previous_actions[1], previous_actions[2], previous_actions[3], previous_actions[4], previous_actions[5]}}
    }};
    constexpr std::array<size_t, kTermCount> widths = {3, 3, 3, 6, 6, 6};
    constexpr std::array<std::array<float, 6>, kTermCount> scales = {{
        {{1.0f, 1.0f, 0.25f, 0.0f, 0.0f, 0.0f}},
        {{0.25f, 0.25f, 0.25f, 0.0f, 0.0f, 0.0f}},
        {{1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f}},
        {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}},
        {{0.05f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f}},
        {{0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f}}
    }};
    for (size_t term = 0; term < kTermCount; ++term)
        for (size_t c = 0; c < widths[term]; ++c)
            f[term][c] = raw[term][c] * scales[term][c];
    return f;
}

Observation flatten(const std::array<Frame, kHistory>& history)
{
    Observation out{};
    constexpr std::array<size_t, kTermCount> widths = {3, 3, 3, 6, 6, 6};
    size_t offset = 0;
    for (size_t term = 0; term < kTermCount; ++term)
        for (size_t h = 0; h < kHistory; ++h)
            for (size_t c = 0; c < widths[term]; ++c) {
                if (offset >= out.size()) throw std::runtime_error("oracle flatten overflow");
                out[offset++] = history[h][term][c];
            }
    if (offset != out.size()) throw std::runtime_error("oracle flatten length mismatch");
    return out;
}

Actions runOracle(Ort::Session& session, const Observation& observation)
{
    Observation owned_input = observation;
    const Observation input_before_run = owned_input;
    std::array<int64_t, 2> shape = {1, 81};
    auto input = Ort::Value::CreateTensor<float>(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault),
                                                  owned_input.data(), owned_input.size(), shape.data(), shape.size());
    const char* input_names[] = {"obs"};
    const char* output_names[] = {"actions"};
    auto results = session.Run(Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
    if (std::memcmp(owned_input.data(), input_before_run.data(), sizeof(owned_input)) != 0)
        throw std::runtime_error("ORT modified the direct-reference input buffer");
    const auto info = results[0].GetTensorTypeAndShapeInfo();
    if (info.GetShape() != std::vector<int64_t>{1, 6} || info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
        throw std::runtime_error("direct ONNX oracle returned unexpected output shape/type");
    Actions result{};
    std::copy_n(results[0].GetTensorMutableData<float>(), result.size(), result.begin());
    return result;
}

void compareObservationBits(const Observation& expected, const Observation& actual)
{
    for (size_t i = 0; i < expected.size(); ++i) {
        if (!std::isfinite(expected[i]) || !std::isfinite(actual[i]))
            throw std::runtime_error("non-finite observation at index " + std::to_string(i));
        if (std::memcmp(&expected[i], &actual[i], sizeof(float)) != 0)
            throw std::runtime_error("bitwise observation mismatch at index " + std::to_string(i));
    }
}

void compareResult(OnnxController& controller, const Actions& expected, const Actions& stand_pos)
{
    const auto log = controller.getLog();
    if (log.size() != 27) throw std::runtime_error("expected 27 controller log values");
    for (size_t i = 0; i < 6; ++i) {
        requireClose(log[21 + i], expected[i], "raw action");
        requireClose(controller.jpos_des[i], stand_pos[i] + kActionScale[i] * expected[i], "desired target");
    }
}
} // namespace

int main()
{
    try {
        const std::filesystem::path source_config(POLICY_PARAMS_FILE);
        const PolicyCfg source_policy_config(source_config.string());
        TemporaryDirectory temp;
        const auto offset_config = temp.path / "offset.yaml";
        const auto missing_offset_config = temp.path / "missing_offset.yaml";
        const auto malformed_config = temp.path / "malformed.yaml";
        const auto nonfinite_config = temp.path / "nonfinite.yaml";
        const Actions offsets = {0.011f, -0.022f, 0.033f, -0.044f, 0.055f, -0.066f};
        std::ostringstream offset_yaml;
        offset_yaml << "default_joint_pos_offset: [";
        for (size_t i = 0; i < offsets.size(); ++i) {
            if (i) offset_yaml << ", ";
            offset_yaml << offsets[i];
        }
        offset_yaml << "]";
        writePolicyConfig(offset_config, source_policy_config.policy_name, offset_yaml.str());
        writePolicyConfig(missing_offset_config, source_policy_config.policy_name);
        const PolicyCfg missing_offset_policy(missing_offset_config.string());
        requireArrayClose(missing_offset_policy.default_joint_pos_offset, Actions{}, "missing offset defaults to zero");
        requireOffsetConfigRejected(malformed_config, source_policy_config.policy_name,
            "default_joint_pos_offset: [0.1, 0.2, 0.3, 0.4, 0.5]", "Expected six YAML values for default_joint_pos_offset");
        requireOffsetConfigRejected(nonfinite_config, source_policy_config.policy_name,
            "default_joint_pos_offset: [0.1, 0.2, 0.3, 0.4, 0.5, .nan]", "Non-finite YAML value for default_joint_pos_offset");

        const PolicyCfg policy_config(offset_config.string());
        const auto model = std::filesystem::weakly_canonical(
            source_config.parent_path() / ".." / "models" / policy_config.policy_name);

        OnnxController default_controller(offset_config.string());
        default_controller.loadParam();
        default_controller.loadPolicy();
        requireArrayClose(default_controller.stand_pos, kDefaultPose, "default interface ignores offset");
        OnnxController lo_controller(offset_config.string(), "lo");
        lo_controller.loadParam();
        lo_controller.loadPolicy();
        requireArrayClose(lo_controller.stand_pos, kDefaultPose, "explicit lo ignores offset");

        Actions expected_shifted_pose{};
        for (size_t i = 0; i < expected_shifted_pose.size(); ++i)
            expected_shifted_pose[i] = kDefaultPose[i] + offsets[i];
        OnnxController controller(offset_config.string(), "eth0");
        controller.loadParam();
        controller.loadPolicy();
        requireArrayClose(controller.stand_pos, expected_shifted_pose, "eth0 applies offset");
        controller.loadPolicy();
        requireArrayClose(controller.stand_pos, expected_shifted_pose, "repeated load does not accumulate offset");

        Ort::Env oracle_env(ORT_LOGGING_LEVEL_WARNING, "onnx_policy_oracle");
        Ort::SessionOptions options;
        Ort::Session oracle(oracle_env, model.c_str(), options);
        Ort::Session oracle_second(oracle_env, model.c_str(), options);

        BasicRobotInterface robot;
        robot.projected_gravity = {0.0f, 0.0f, -1.0f};
        robot.gyro = {0.0f, 0.0f, 0.0f};
        robot.jpos = controller.stand_pos;
        robot.jvel = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        Gamepad gamepad;
        // A small negative x command yields deterministic targets within the
        // configured hip bounds for this shipped policy.
        gamepad.ly = -0.5f; gamepad.ry = 0.0f; gamepad.lx = 0.0f; gamepad.rx = 0.0f;
        Actions previous{};
        controller.getInput(robot, gamepad);
        const auto snapshot = controller.getLog();
        controller.reset(robot);
        if (controller.getLog() != snapshot) throw std::runtime_error("reset did not preserve current input snapshot");
        // Defer warm-up until after deterministic reference comparisons; it is
        // still exercised below without affecting the live observation sequence.

        Frame first = makeExpectedFrame(robot, gamepad, previous, controller.stand_pos);
        std::array<Frame, kHistory> first_history = {first, first, first}; // first-frame backfill
        const Observation expected_first_input = flatten(first_history);
        const Actions first_expected = runOracle(oracle, expected_first_input);
        for (int repeat = 0; repeat < 3; ++repeat) {
            const Actions same_session = runOracle(oracle, expected_first_input);
            const Actions other_session = runOracle(oracle_second, expected_first_input);
            for (size_t i = 0; i < first_expected.size(); ++i) {
                requireClose(same_session[i], first_expected[i], "repeated same-session action");
                requireClose(other_session[i], first_expected[i], "repeated cross-session action");
            }
        }
        try { controller.calculate(); }
        catch (const std::exception& error) {
            const Observation expected_input = flatten(first_history);
            compareObservationBits(expected_input, controller.lastPolicyInput());
            const Actions replay = runOracle(oracle, controller.lastPolicyInput());
            for (size_t i = 0; i < replay.size(); ++i) {
                if (!std::isfinite(controller.lastPolicyOutput()[i]) ||
                    std::abs(replay[i] - first_expected[i]) > 1e-5f ||
                    std::abs(replay[i] - controller.lastPolicyOutput()[i]) > 1e-5f)
                    throw std::runtime_error("direct ORT replay mismatch at action " + std::to_string(i) +
                        ": expected=" + std::to_string(first_expected[i]) +
                        " replay=" + std::to_string(replay[i]) +
                        " controller=" + std::to_string(controller.lastPolicyOutput()[i]) +
                        " original controller error=" + error.what());
            }
            throw;
        }
        compareObservationBits(expected_first_input, controller.lastPolicyInput());
        compareResult(controller, first_expected, controller.stand_pos);
        previous = first_expected; // raw outputs become the next observation's action term

        robot.projected_gravity = {0.01f, -0.01f, -0.9999f};
        robot.gyro = {0.01f, -0.02f, 0.01f};
        robot.jpos = controller.stand_pos;
        const Actions position_delta = {0.005f, 0.005f, 0.005f, -0.005f, -0.005f, -0.005f};
        for (size_t i = 0; i < robot.jpos.size(); ++i) robot.jpos[i] += position_delta[i];
        robot.jvel = {0.005f, -0.005f, 0.01f, -0.005f, 0.005f, -0.01f};
        gamepad.ly = 0.01f; gamepad.ry = 0.0f; gamepad.lx = 0.01f; gamepad.rx = -0.01f;
        controller.getInput(robot, gamepad);
        const Frame second = makeExpectedFrame(robot, gamepad, previous, controller.stand_pos);
        const std::array<Frame, kHistory> second_history = {first, first, second};
        const Observation expected_second_input = flatten(second_history);
        const Actions second_expected = runOracle(oracle, expected_second_input);
        controller.calculate();
        compareObservationBits(expected_second_input, controller.lastPolicyInput());
        compareResult(controller, second_expected, controller.stand_pos);
        controller.dummyCalculate();
        for (float value : controller.getLog())
            if (!std::isfinite(value)) throw std::runtime_error("non-finite controller log value");

        // Reproduce the earlier neutral-frame regression using an owned oracle
        // buffer and compare it against the exact bytes sent by the controller.
        robot.projected_gravity = {0.0f, 0.0f, -1.0f};
        robot.gyro = {0.0f, 0.0f, 0.0f};
        robot.jpos = controller.stand_pos;
        robot.jvel = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        gamepad.ly = 0.0f; gamepad.ry = 0.0f; gamepad.lx = 0.0f; gamepad.rx = 0.0f;
        controller.getInput(robot, gamepad);
        controller.reset(robot);
        const Frame neutral_frame = makeExpectedFrame(robot, gamepad, Actions{}, controller.stand_pos);
        const Observation neutral_input = flatten({neutral_frame, neutral_frame, neutral_frame});
        const Actions neutral_expected = runOracle(oracle, neutral_input);
        controller.calculate();
        compareObservationBits(neutral_input, controller.lastPolicyInput());
        const Actions neutral_replay = runOracle(oracle_second, controller.lastPolicyInput());
        for (size_t i = 0; i < neutral_expected.size(); ++i) {
            requireClose(neutral_replay[i], neutral_expected[i], "neutral repeated-session action");
            requireClose(controller.lastPolicyOutput()[i], neutral_expected[i], "neutral controller action");
        }
        compareResult(controller, neutral_expected, controller.stand_pos);

        // Compare a synthetic all-zero observation and all resulting targets
        // without assuming anything about their range relative to YAML limits.
        robot.projected_gravity = {0.0f, 0.0f, 0.0f};
        robot.gyro = {0.0f, 0.0f, 0.0f};
        robot.jpos = controller.stand_pos;
        robot.jvel = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        gamepad.ly = 0.0f; gamepad.ry = 0.0f; gamepad.lx = 0.0f; gamepad.rx = 0.0f;
        controller.getInput(robot, gamepad);
        controller.reset(robot);
        const Frame zero_frame = makeExpectedFrame(robot, gamepad, Actions{}, controller.stand_pos);
        const Observation zero_input = flatten({zero_frame, zero_frame, zero_frame});
        const Actions zero_expected = runOracle(oracle, zero_input);
        controller.calculate();
        compareObservationBits(zero_input, controller.lastPolicyInput());
        const Actions zero_replay = runOracle(oracle_second, controller.lastPolicyInput());
        for (size_t i = 0; i < zero_replay.size(); ++i) {
            requireClose(zero_replay[i], zero_expected[i], "zero-frame oracle replay");
            requireClose(controller.lastPolicyOutput()[i], zero_expected[i], "zero-frame controller action");
        }
        compareResult(controller, zero_expected, controller.stand_pos);

        std::cout << "ONNX policy oracle smoke test passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ONNX policy oracle smoke test failed: " << error.what() << '\n';
        return 1;
    }
}
