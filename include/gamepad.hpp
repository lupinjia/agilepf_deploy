/**
 * @file gamepad.hpp
 * @author lupinjia (jason_1120202397@163.com)
 * @brief Gamepad class for processing joystick data from the DDS topic.
 * @version 0.1
 * @date 2026-10-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#pragma once

#include <cmath>
#include <idl/agilepf/JoystickData/JoystickData.hpp>

/**
 * @brief Button class, defining 3 states of a button (pressed, on_press, on_release).
 * 
 */
class Button
{
public:
    Button() {}

    /**
     * @brief Update the button state.
     * 
     * @param state The new state of the button.
     */
    void update(bool state)
    {
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }

    bool pressed = false;    // pressed会持续
    bool on_press = false;   // on_press和on_release都是一瞬间
    bool on_release = false;
};

/**
 * @brief Gamepad class, processing joystick data.
 * 
 */
class Gamepad
{
public:
    Gamepad() {}

    /**
     * @brief Update the gamepad state.
     * 
     * @param joystick_data The joystick data to process.
     */
    void update(const legged::idl::agilepf::JoystickData &joystick_data)
    {
        lx = lx * (1 - smooth) + (std::fabs(joystick_data.left_stick_x()) < dead_zone ? 0.0 : joystick_data.left_stick_x()) * smooth;
        rx = rx * (1 - smooth) + (std::fabs(joystick_data.right_stick_x()) < dead_zone ? 0.0 : joystick_data.right_stick_x()) * smooth;
        ry = ry * (1 - smooth) + (std::fabs(joystick_data.right_stick_y()) < dead_zone ? 0.0 : joystick_data.right_stick_y()) * smooth;
        ly = ly * (1 - smooth) + (std::fabs(joystick_data.left_stick_y()) < dead_zone ? 0.0 : joystick_data.left_stick_y()) * smooth;

        R1.update(joystick_data.r1_button());
        L1.update(joystick_data.l1_button());
        start.update(joystick_data.start_button());
        select.update(joystick_data.select_button());
        A.update(joystick_data.a_button());
        B.update(joystick_data.b_button());
        X.update(joystick_data.x_button());
        Y.update(joystick_data.y_button());
        up.update(joystick_data.up_button());
        right.update(joystick_data.right_button());
        down.update(joystick_data.down_button());
        left.update(joystick_data.left_button());
    }

    // left stick x axis
    float lx = 0.;
    // right stick x axis
    float rx = 0.;
    // right stick y axis
    float ry = 0.;
    // left stick y axis
    float ly = 0.;

    float smooth = 0.03;
    float dead_zone = 0.01;

    Button R1;
    Button L1;
    Button start;
    Button select;
    Button A;
    Button B;
    Button X;
    Button Y;
    Button up;
    Button right;
    Button down;
    Button left;
};