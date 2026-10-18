#ifndef __USER_CONTROLLER_HPP__
#define __USER_CONTROLLER_HPP__

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <onnxruntime_cxx_api.h>
#include "robot_interface.hpp"
#include "cfg.hpp"
#include "gamepad.hpp"

class BasicUserController
{
public:
    virtual ~BasicUserController() = default;
    virtual void loadParam() = 0;
    virtual void loadPolicy() = 0;
    virtual void reset(BasicRobotInterface&) = 0;
    virtual void getInput(BasicRobotInterface&, Gamepad&) = 0;
    virtual void dummyCalculate() = 0;
    virtual void calculate() = 0;
    virtual std::vector<float> getLog() = 0;
    void save_jpos(BasicRobotInterface& robot) { std::copy(robot.jpos.begin(), robot.jpos.end(), start_pos.begin()); }

    float dt{}, stand_kp{}, stand_kd{}, ctrl_kp{}, ctrl_kd{}, rpy_offset_increment{};
    std::array<float, 6> start_pos{}, stand_pos{}, jpos_des{}, lower_limit{}, upper_limit{};
};

class OnnxController final : public BasicUserController
{
    static constexpr size_t JointCount = 6;
    static constexpr size_t TermCount = 6;
    static constexpr size_t History = 3;
    static constexpr size_t ObsDim = 81;
    using JointValues = std::array<float, JointCount>;
    using Frame = std::array<std::array<float, 6>, TermCount>;

    struct ModelMetadata {
        std::array<std::string, 6> joint_names;
        JointValues stiffness{}, damping{}, default_pos{}, action_scale{};
        std::array<std::array<float, 6>, TermCount> scales{}, clip_low{}, clip_high{};
        std::array<Frame, History> history{};
    };

public:
    explicit OnnxController(const std::string& config_filename)
        : config_filename_(config_filename), env_(ORT_LOGGING_LEVEL_WARNING, "OnnxController")
    {
        cmd.fill(0); gravity = {0, 0, -1}; gyro.fill(0); jpos.fill(0); jvel.fill(0); actions.fill(0);
        jpos_des.fill(0);
    }

    void loadParam() override
    {
        const PolicyCfg cfg(config_filename_);
        dt = cfg.dt; policy_name_ = cfg.policy_name; stand_kp = cfg.stand_kp; stand_kd = cfg.stand_kd;
        cmd_lin_x_scale_ = cfg.cmd_lin_x_scale; cmd_lin_y_scale_ = cfg.cmd_lin_y_scale;
        cmd_ang_z_scale_ = cfg.cmd_ang_z_scale; min_lin_x_ = cfg.min_lin_x; max_lin_x_ = cfg.max_lin_x;
        rpy_offset_increment = cfg.rpy_offset_increment; start_pos = cfg.start_pos;
        lower_limit = cfg.lower_limit; upper_limit = cfg.upper_limit;
        params_loaded_ = true;
    }

    void loadPolicy() override
    {
        if (!params_loaded_) throw std::runtime_error("OnnxController::loadParam must precede loadPolicy");
        const auto model_path = resolveModelPath();
        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), session_options_);
        parseMetadata();
        validateGraph();
        stand_pos = metadata_.default_pos;
        for (size_t i = 0; i < JointCount; ++i) {
            if (std::abs(metadata_.stiffness[i] - metadata_.stiffness[0]) > 1e-5f ||
                std::abs(metadata_.damping[i] - metadata_.damping[0]) > 1e-5f)
                throw std::runtime_error("ONNX metadata joint gains are nonuniform; scalar robot API cannot represent them");
        }
        ctrl_kp = metadata_.stiffness[0]; ctrl_kd = metadata_.damping[0];
        input_name_ = metadata_string("onnx_input_names");
        output_name_ = metadata_string("onnx_output_names");
    }

    void getInput(BasicRobotInterface& robot, Gamepad& gamepad) override
    {
        cmd[0] = std::clamp(gamepad.ly * cmd_lin_x_scale_ + gamepad.ry * cmd_lin_x_scale_, min_lin_x_, max_lin_x_);
        cmd[1] = -gamepad.lx * cmd_lin_y_scale_;
        cmd[2] = -gamepad.rx * cmd_ang_z_scale_;
        std::copy(robot.projected_gravity.begin(), robot.projected_gravity.end(), gravity.begin());
        std::copy(robot.gyro.begin(), robot.gyro.end(), gyro.begin());
        std::copy(robot.jpos.begin(), robot.jpos.end(), jpos.begin());
        std::copy(robot.jvel.begin(), robot.jvel.end(), jvel.begin());
        ensureFinite(cmd, "joystick command"); ensureFinite(gravity, "projected gravity");
        ensureFinite(gyro, "gyro"); ensureFinite(jpos, "joint position"); ensureFinite(jvel, "joint velocity");
    }

    void reset(BasicRobotInterface&) override
    {
        for (auto& frame : metadata_.history) frame = Frame{};
        // Keep the latest getInput() snapshot: reset is invoked after getInput()
        // on the transition into CTRL, and that snapshot must feed its first frame.
        actions.fill(0);
        history_count_ = 0; jpos_des = stand_pos;
    }

    void dummyCalculate() override
    {
        requirePolicy();
        std::array<float, ObsDim> scratch{};
        infer(scratch.data()); // warm-up uses private zero input; no controller state is touched
    }

    void calculate() override
    {
        requirePolicy();
        const Frame current = makeFrame();
        if (history_count_ == 0) metadata_.history.fill(current);
        else {
            for (size_t h = 0; h + 1 < History; ++h) metadata_.history[h] = metadata_.history[h + 1];
            metadata_.history[History - 1] = current;
        }
        history_count_ = std::min(history_count_ + 1, History);
        std::array<float, ObsDim> input{};
        size_t offset = 0;
        const std::array<size_t, TermCount> widths = {3, 3, 3, 6, 6, 6};
        for (size_t term = 0; term < TermCount; ++term)
            for (size_t h = 0; h < History; ++h)
                for (size_t component = 0; component < widths[term]; ++component) {
                    if (offset >= input.size()) throw std::runtime_error("Observation flatten exceeded ONNX input buffer");
                    input[offset++] = metadata_.history[h][term][component];
                }
        if (offset != ObsDim) throw std::runtime_error("Observation flatten length does not match ONNX input dimension");
        last_input_ = input;
        std::array<float, JointCount> output{};
        infer(input.data(), output.data());
        last_output_ = output;
        JointValues candidate_targets{};
        for (size_t i = 0; i < JointCount; ++i) {
            if (!std::isfinite(output[i])) throw std::runtime_error("ONNX policy produced a non-finite action");
            candidate_targets[i] = stand_pos[i] + metadata_.action_scale[i] * output[i];
            if (!std::isfinite(candidate_targets[i])) throw std::runtime_error("ONNX policy produced a non-finite desired joint position");
        }
        // Commit only after all raw actions and computed targets are finite.
        actions = output;
        jpos_des = candidate_targets;
    }

    std::vector<float> getLog() override
    {
        std::vector<float> log; log.reserve(27);
        log.insert(log.end(), cmd.begin(), cmd.end()); log.insert(log.end(), gravity.begin(), gravity.end());
        log.insert(log.end(), gyro.begin(), gyro.end()); log.insert(log.end(), jpos.begin(), jpos.end());
        log.insert(log.end(), jvel.begin(), jvel.end()); log.insert(log.end(), actions.begin(), actions.end());
        return log;
    }

    const std::array<float, ObsDim>& lastPolicyInput() const { return last_input_; }
    const JointValues& lastPolicyOutput() const { return last_output_; }

private:
    std::string config_filename_, policy_name_, input_name_, output_name_;
    float cmd_lin_x_scale_{}, cmd_lin_y_scale_{}, cmd_ang_z_scale_{}, min_lin_x_{}, max_lin_x_{};
    bool params_loaded_{};
    size_t history_count_{};
    std::array<float, 3> cmd{}, gravity{}, gyro{};
    JointValues jpos{}, jvel{}, actions{};
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    ModelMetadata metadata_{};
    std::array<float, ObsDim> last_input_{};
    JointValues last_output_{};

    template <typename Values> static void ensureFinite(const Values& values, const char* label)
    {
        for (float value : values) if (!std::isfinite(value)) throw std::runtime_error(std::string("Non-finite ") + label);
    }
    static std::vector<std::string> split(const std::string& text, char delimiter)
    {
        std::vector<std::string> out; size_t begin = 0;
        while (begin <= text.size()) { const size_t end = text.find(delimiter, begin); out.push_back(text.substr(begin, end - begin)); if (end == std::string::npos) break; begin = end + 1; }
        return out;
    }
    static float parseNumber(const std::string& text, const std::string& key, bool allow_inf = false)
    {
        size_t used = 0; float value;
        try { value = std::stof(text, &used); } catch (...) { throw std::runtime_error("Invalid numeric ONNX metadata " + key + ": " + text); }
        if (used != text.size() || (!std::isfinite(value) && !(allow_inf && std::isinf(value))))
            throw std::runtime_error("Invalid numeric ONNX metadata " + key + ": " + text);
        return value;
    }
    std::string metadata_string(const std::string& key)
    {
        Ort::AllocatorWithDefaultOptions allocator;
        auto metadata = session_->GetModelMetadata();
        auto value = metadata.LookupCustomMetadataMapAllocated(key.c_str(), allocator);
        if (!value || !*value) throw std::runtime_error("ONNX model missing required custom metadata key: " + key);
        return value.get();
    }
    std::filesystem::path resolveModelPath() const
    {
        const std::filesystem::path configured(policy_name_);
        if (configured.is_absolute() && std::filesystem::exists(configured)) return configured;
        std::vector<std::filesystem::path> candidates;
        candidates.push_back(std::filesystem::path(config_filename_).parent_path() / ".." / "models" / configured.filename());
        candidates.push_back(std::filesystem::current_path() / ".." / "models" / configured.filename());
        for (const auto& p : candidates) if (std::filesystem::exists(p)) return std::filesystem::weakly_canonical(p);
        throw std::runtime_error("Cannot locate ONNX model '" + policy_name_ + "' relative to config or build directory");
    }
    template <size_t N> std::array<float, N> numericVector(const std::string& key, bool allow_inf = false)
    {
        const auto value = metadata_string(key);
        const auto tokens = split(value, value.find(';') == std::string::npos ? ',' : ';');
        if (tokens.size() != N) throw std::runtime_error("ONNX metadata " + key + " must contain " + std::to_string(N) + " values");
        std::array<float, N> out{};
        for (size_t i = 0; i < N; ++i) out[i] = parseNumber(tokens[i], key, allow_inf);
        return out;
    }
    void parseMetadata()
    {
        const std::array<std::string, 6> expected = {"left_hip_joint", "left_thigh_joint", "left_calf_joint", "right_hip_joint", "right_thigh_joint", "right_calf_joint"};
        const auto names = split(metadata_string("joint_names"), ',');
        if (names.size() != JointCount) throw std::runtime_error("ONNX joint_names must contain six names");
        for (size_t i = 0; i < JointCount; ++i) {
            metadata_.joint_names[i] = names[i];
            if (names[i] != expected[i]) throw std::runtime_error("ONNX joint_names order does not match robot motor order at index " + std::to_string(i));
        }
        metadata_.stiffness = numericVector<6>("joint_stiffness");
        metadata_.damping = numericVector<6>("joint_damping");
        metadata_.default_pos = numericVector<6>("default_joint_pos");
        metadata_.action_scale = numericVector<6>("action_scale");
        for (size_t i = 0; i < JointCount; ++i) {
            if (!std::isfinite(metadata_.action_scale[i]) || metadata_.action_scale[i] <= 0.0f)
                throw std::runtime_error("ONNX action_scale must be finite and positive at joint " + std::to_string(i));
            if (!std::isfinite(metadata_.default_pos[i]) || !std::isfinite(metadata_.stiffness[i]) ||
                !std::isfinite(metadata_.damping[i]))
                throw std::runtime_error("Non-finite ONNX joint metadata at index " + std::to_string(i));
            if (!std::isfinite(lower_limit[i]) || !std::isfinite(upper_limit[i]) || lower_limit[i] > upper_limit[i])
                throw std::runtime_error("Invalid configured policy target bounds at joint " + std::to_string(i));
        }
        if (split(metadata_string("onnx_input_names"), ',') != std::vector<std::string>{"obs"} ||
            split(metadata_string("onnx_output_names"), ',') != std::vector<std::string>{"actions"} ||
            split(metadata_string("onnx_input_shapes"), ';') != std::vector<std::string>{"1.000", "81.000"} ||
            split(metadata_string("onnx_output_shapes"), ';') != std::vector<std::string>{"1.000", "6.000"})
            throw std::runtime_error("Unsupported ONNX input/output name or shape metadata");
        // onnx_input_groups="actor" names the training group; exported term
        // metadata is namespaced by the graph input name, which is "obs".
        if (parseNumber(metadata_string("obs_group.obs.history_length"), "obs history_length") != 3.f ||
            metadata_string("obs_group.obs.flatten_history_dim") != "True")
            throw std::runtime_error("Unsupported obs history configuration in ONNX metadata");
        const auto terms = split(metadata_string("obs_group.obs.term_names"), ',');
        const std::array<std::string, 6> expected_terms = {"command", "base_ang_vel", "projected_gravity", "joint_pos", "joint_vel", "actions"};
        if (terms.size() != TermCount || !std::equal(terms.begin(), terms.end(), expected_terms.begin()))
            throw std::runtime_error("Unsupported obs term names/order in ONNX metadata");
        const auto histories = split(metadata_string("obs_group.obs.term_history_lengths"), ',');
        if (histories.size() != TermCount) throw std::runtime_error("Expected six term_history_lengths");
        for (const auto& item : histories) if (parseNumber(item, "term_history_lengths") != 3.f) throw std::runtime_error("Each supported term history length must be 3");
        const auto clip_groups = split(metadata_string("obs_group.obs.term_clips"), ',');
        const auto scale_groups = split(metadata_string("obs_group.obs.term_scales"), ',');
        if (clip_groups.size() != TermCount || scale_groups.size() != TermCount) throw std::runtime_error("Term clips/scales must each contain six terms");
        const std::array<size_t, 6> widths = {3, 3, 3, 6, 6, 6};
        for (size_t term = 0; term < TermCount; ++term) {
            const auto bounds = split(clip_groups[term], ';');
            if (bounds.size() != 2) throw std::runtime_error("Malformed clip bounds for term " + expected_terms[term]);
            float low = parseNumber(bounds[0], "term_clips", true), high = parseNumber(bounds[1], "term_clips", true);
            if (low > high) throw std::runtime_error("Invalid clip bound order for " + expected_terms[term]);
            const auto scales = split(scale_groups[term], ';');
            if (scales.size() != 1 && scales.size() != widths[term])
                throw std::runtime_error("Scale width mismatch for term " + expected_terms[term]);
            for (size_t c = 0; c < widths[term]; ++c) {
                metadata_.clip_low[term][c] = low; metadata_.clip_high[term][c] = high;
                metadata_.scales[term][c] = parseNumber(scales[scales.size() == 1 ? 0 : c], "term_scales");
            }
        }
    }
    void validateGraph()
    {
        if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1) throw std::runtime_error("ONNX graph must have one input and one output");
        Ort::AllocatorWithDefaultOptions allocator;
        auto input_name = session_->GetInputNameAllocated(0, allocator), output_name = session_->GetOutputNameAllocated(0, allocator);
        if (input_name.get() != std::string("obs") || output_name.get() != std::string("actions")) throw std::runtime_error("ONNX graph node names do not match required metadata names");
        auto check = [](const Ort::TypeInfo& info, const std::array<int64_t, 2>& wanted, const char* label) {
            auto tensor = info.GetTensorTypeAndShapeInfo();
            if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || tensor.GetShape() != std::vector<int64_t>(wanted.begin(), wanted.end()))
                throw std::runtime_error(std::string("ONNX graph ") + label + " must be float32 with shape [" + std::to_string(wanted[0]) + "," + std::to_string(wanted[1]) + "]");
        };
        check(session_->GetInputTypeInfo(0), {1, 81}, "input"); check(session_->GetOutputTypeInfo(0), {1, 6}, "output");
    }
    Frame makeFrame() const
    {
        Frame frame{};
        const std::array<float, 6> raw[] = {
            {cmd[0], cmd[1], cmd[2], 0, 0, 0}, {gyro[0], gyro[1], gyro[2], 0, 0, 0},
            {gravity[0], gravity[1], gravity[2], 0, 0, 0},
            {jpos[0]-stand_pos[0], jpos[1]-stand_pos[1], jpos[2]-stand_pos[2], jpos[3]-stand_pos[3], jpos[4]-stand_pos[4], jpos[5]-stand_pos[5]},
            {jvel[0], jvel[1], jvel[2], jvel[3], jvel[4], jvel[5]},
            {actions[0], actions[1], actions[2], actions[3], actions[4], actions[5]}};
        const std::array<size_t, 6> widths = {3, 3, 3, 6, 6, 6};
        for (size_t t = 0; t < TermCount; ++t) for (size_t c = 0; c < widths[t]; ++c) {
            const float clipped = std::clamp(raw[t][c], metadata_.clip_low[t][c], metadata_.clip_high[t][c]);
            frame[t][c] = clipped * metadata_.scales[t][c];
            if (!std::isfinite(frame[t][c])) throw std::runtime_error("Non-finite normalized observation");
        }
        return frame;
    }
    void requirePolicy() const { if (!session_) throw std::runtime_error("ONNX policy is not loaded"); }
    void infer(float* input, float* output = nullptr)
    {
        if (output && std::memcmp(input, last_input_.data(), sizeof(last_input_)) != 0)
            throw std::runtime_error("ONNX input bytes changed between flattening and Run");
        std::array<int64_t, 2> input_shape{1, 81};
        const auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto value = Ort::Value::CreateTensor<float>(memory_info, input, ObsDim, input_shape.data(), input_shape.size());
        const char* in_names[] = {input_name_.c_str()}; const char* out_names[] = {output_name_.c_str()};
        auto result = session_->Run(Ort::RunOptions{nullptr}, in_names, &value, 1, out_names, 1);
        if (output) {
            if (!result[0].IsTensor() || result[0].GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
                result[0].GetTensorTypeAndShapeInfo().GetShape() != std::vector<int64_t>{1, 6})
                throw std::runtime_error("ONNX inference returned an invalid action tensor");
            const float* output_data = result[0].GetTensorMutableData<float>();
            for (size_t i = 0; i < JointCount; ++i) {
                if (!std::isfinite(output_data[i]))
                    throw std::runtime_error("ONNX inference produced a non-finite action");
            }
            std::copy_n(output_data, JointCount, output);
        }
    }
};

#endif
