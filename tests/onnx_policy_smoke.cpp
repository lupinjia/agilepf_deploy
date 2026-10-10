#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
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

void requireClose(float actual, float expected, const char* label)
{
    if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-5f)
        throw std::runtime_error(std::string(label) + " differs from direct ONNX reference");
}

Frame makeExpectedFrame(const BasicRobotInterface& robot, const Gamepad& pad, const Actions& previous_actions)
{
    Frame f{};
    const std::array<std::array<float, 6>, kTermCount> raw = {{
        {{pad.ly + pad.ry, -pad.lx * 0.4f, -pad.rx, 0, 0, 0}},
        {{robot.gyro[0], robot.gyro[1], robot.gyro[2], 0, 0, 0}},
        {{robot.projected_gravity[0], robot.projected_gravity[1], robot.projected_gravity[2], 0, 0, 0}},
        {{robot.jpos[0], robot.jpos[1] - 0.5f, robot.jpos[2] + 0.5f, robot.jpos[3], robot.jpos[4] - 0.5f, robot.jpos[5] + 0.5f}},
        {{robot.jvel[0], robot.jvel[1], robot.jvel[2], robot.jvel[3], robot.jvel[4], robot.jvel[5]}},
        {{previous_actions[0], previous_actions[1], previous_actions[2], previous_actions[3], previous_actions[4], previous_actions[5]}}
    }};
    constexpr std::array<size_t, kTermCount> widths = {3, 3, 3, 6, 6, 6};
    constexpr std::array<float, kTermCount> scales = {0.25f, 0.25f, 1.0f, 1.0f, 0.05f, 0.25f};
    for (size_t term = 0; term < kTermCount; ++term)
        for (size_t c = 0; c < widths[term]; ++c)
            f[term][c] = raw[term][c] * (term == 0 && c < 2 ? 1.0f : scales[term]);
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

void compareResult(OnnxController& controller, const Actions& expected)
{
    const auto log = controller.getLog();
    if (log.size() != 27) throw std::runtime_error("expected 27 controller log values");
    for (size_t i = 0; i < 6; ++i) {
        requireClose(log[21 + i], expected[i], "raw action");
        requireClose(controller.jpos_des[i], kDefaultPose[i] + kActionScale[i] * expected[i], "desired target");
    }
}
} // namespace

int main()
{
    try {
        const std::filesystem::path config(POLICY_PARAMS_FILE);
        const auto model = std::filesystem::weakly_canonical(config.parent_path() / ".." / "models" / "2026-10-09_18-29-35_flat.onnx");
        OnnxController controller(config.string());
        controller.loadParam();
        controller.loadPolicy();

        Ort::Env oracle_env(ORT_LOGGING_LEVEL_WARNING, "onnx_policy_oracle");
        Ort::SessionOptions options;
        Ort::Session oracle(oracle_env, model.c_str(), options);
        Ort::Session oracle_second(oracle_env, model.c_str(), options);

        BasicRobotInterface robot;
        robot.projected_gravity = {0.0f, 0.0f, -1.0f};
        robot.gyro = {0.0f, 0.0f, 0.0f};
        robot.jpos = {0.0f, 0.5f, -0.5f, 0.0f, 0.5f, -0.5f};
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

        Frame first = makeExpectedFrame(robot, gamepad, previous);
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
        compareResult(controller, first_expected);
        previous = first_expected; // raw outputs become the next observation's action term

        robot.projected_gravity = {0.01f, -0.01f, -0.9999f};
        robot.gyro = {0.01f, -0.02f, 0.01f};
        robot.jpos = {0.005f, 0.505f, -0.495f, -0.005f, 0.495f, -0.505f};
        robot.jvel = {0.005f, -0.005f, 0.01f, -0.005f, 0.005f, -0.01f};
        gamepad.ly = 0.01f; gamepad.ry = 0.0f; gamepad.lx = 0.01f; gamepad.rx = -0.01f;
        controller.getInput(robot, gamepad);
        const Frame second = makeExpectedFrame(robot, gamepad, previous);
        const std::array<Frame, kHistory> second_history = {first, first, second};
        const Observation expected_second_input = flatten(second_history);
        const Actions second_expected = runOracle(oracle, expected_second_input);
        controller.calculate();
        compareObservationBits(expected_second_input, controller.lastPolicyInput());
        compareResult(controller, second_expected);
        controller.dummyCalculate();
        for (float value : controller.getLog())
            if (!std::isfinite(value)) throw std::runtime_error("non-finite controller log value");

        // Reproduce the earlier neutral-frame regression using an owned oracle
        // buffer and compare it against the exact bytes sent by the controller.
        robot.projected_gravity = {0.0f, 0.0f, -1.0f};
        robot.gyro = {0.0f, 0.0f, 0.0f};
        robot.jpos = {0.0f, 0.5f, -0.5f, 0.0f, 0.5f, -0.5f};
        robot.jvel = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        gamepad.ly = 0.0f; gamepad.ry = 0.0f; gamepad.lx = 0.0f; gamepad.rx = 0.0f;
        controller.getInput(robot, gamepad);
        controller.reset(robot);
        const Frame neutral_frame = makeExpectedFrame(robot, gamepad, Actions{});
        const Observation neutral_input = flatten({neutral_frame, neutral_frame, neutral_frame});
        const Actions neutral_expected = runOracle(oracle, neutral_input);
        controller.calculate();
        compareObservationBits(neutral_input, controller.lastPolicyInput());
        const Actions neutral_replay = runOracle(oracle_second, controller.lastPolicyInput());
        for (size_t i = 0; i < neutral_expected.size(); ++i) {
            requireClose(neutral_replay[i], neutral_expected[i], "neutral repeated-session action");
            requireClose(controller.lastPolicyOutput()[i], neutral_expected[i], "neutral controller action");
        }
        compareResult(controller, neutral_expected);

        // The synthetic all-zero observation intentionally produces a target
        // outside the configured bounds; target bounds are not enforced here.
        robot.projected_gravity = {0.0f, 0.0f, 0.0f};
        robot.gyro = {0.0f, 0.0f, 0.0f};
        robot.jpos = {0.0f, 0.5f, -0.5f, 0.0f, 0.5f, -0.5f};
        robot.jvel = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        gamepad.ly = 0.0f; gamepad.ry = 0.0f; gamepad.lx = 0.0f; gamepad.rx = 0.0f;
        controller.getInput(robot, gamepad);
        controller.reset(robot);
        const Frame unsafe_frame = makeExpectedFrame(robot, gamepad, Actions{});
        const Observation unsafe_input = flatten({unsafe_frame, unsafe_frame, unsafe_frame});
        const Actions unsafe_expected = runOracle(oracle, unsafe_input);
        bool oracle_target_violates_limit = false;
        for (size_t i = 0; i < 6; ++i) {
            const float target = kDefaultPose[i] + kActionScale[i] * unsafe_expected[i];
            if (target < controller.lower_limit[i] || target > controller.upper_limit[i]) oracle_target_violates_limit = true;
        }
        if (!oracle_target_violates_limit) throw std::runtime_error("unsafe-frame oracle no longer exercises a configured target limit");
        controller.calculate();
        compareObservationBits(unsafe_input, controller.lastPolicyInput());
        const Actions unsafe_replay = runOracle(oracle_second, controller.lastPolicyInput());
        for (size_t i = 0; i < unsafe_replay.size(); ++i) {
            requireClose(unsafe_replay[i], unsafe_expected[i], "unsafe-frame oracle replay");
            requireClose(controller.lastPolicyOutput()[i], unsafe_expected[i], "unsafe-frame controller action");
        }
        compareResult(controller, unsafe_expected);
        if (controller.jpos_des[0] >= controller.lower_limit[0] && controller.jpos_des[0] <= controller.upper_limit[0])
            throw std::runtime_error("synthetic all-zero policy target was not committed outside joint-0 bounds");
        if (controller.getLog()[21] != unsafe_expected[0])
            throw std::runtime_error("synthetic all-zero raw action was not committed");

        std::cout << "ONNX policy oracle smoke test passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ONNX policy oracle smoke test failed: " << error.what() << '\n';
        return 1;
    }
}
