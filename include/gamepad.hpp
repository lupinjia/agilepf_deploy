#pragma once

#include <cmath>
#include <idl/agilepf/JoystickData/JoystickData.hpp>

class Button
{
public:
    Button() {}

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

class Gamepad
{
public:
    Gamepad() {}

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

    float lx = 0.;
    float rx = 0.;
    float ry = 0.;
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