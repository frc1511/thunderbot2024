#pragma once

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/current.h>

struct PID_t
{
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kff = 0;
    double Kizone = 0;
    units::angular_velocity::degrees_per_second_t MaxVel = 0_deg_per_s;
    units::angular_acceleration::degrees_per_second_squared_t MaxAccel = 0_deg_per_s_sq;
    void clear() 
    {
        Kp = Ki = Kd = Kff = Kizone = 0;
        MaxVel = 0_deg_per_s;
        MaxAccel = 0_deg_per_s_sq;
    }
};

struct PreferencesControls
{
    double MAX_ARM_SPEED = 0.5;
    double MAX_HANG_UP_SPEED = 0.6;
    double MAX_HANG_DOWN_SPEED = 0.4;
    double AXIS_DEADZONE = 0.1;
};
static PreferencesControls PREFERENCE_CONTROLS;

struct PreferencesHang
{
    double MAX_POSTION = 160;
};
static PreferencesHang PREFERENCE_HANG;

struct PreferencesTrajectory
{
        units::meter_t FIELD_X = 16.54175_m;
        units::meter_t FIELD_Y = 8.0137_m;
};
static PreferencesTrajectory PREFERENCE_TRAJECTORY;

struct PreferencesDrive
{
    PID_t PID_XY;
    PID_t PID_THETA;
    units::inch_t ROBOT_WIDTH = 20.1875_in;
    units::inch_t ROBOT_LENGTH = 20.1875_in;

    units::radians_per_second_t DRIVE_AUTO_MAX_ANG_VEL = 6.28_rad_per_s;
    units::radians_per_second_squared_t DRIVE_AUTO_MAX_ANG_ACCEL = 3.14_rad_per_s_sq;

    units::meters_per_second_t DRIVE_MANUAL_MAX_VEL = 5_mps;
    units::degrees_per_second_t DRIVE_MANUAL_MAX_ANG_VEL = 540_deg_per_s;
    units::radians_per_second_squared_t DRIVE_MANUAL_MAX_ANG_ACCEL = 9.42_rad_per_s_sq;

    PreferencesDrive()
    {
        PID_XY.Kp = 3.25;
        PID_XY.Ki = 0.1;

        PID_THETA.Kp = 8.0;
        PID_THETA.Kd = 0.1;
    }
};
static PreferencesDrive PREFERENCE_DRIVE;

struct PreferencesDriveMotor
{
    PID_t PID;
    units::current::ampere_t MAX_AMPERAGE = 40_A;
    double DRIVE_FOOT_TO_ENDODER_FACTOR = 7.76033972;
    double DRIVE_METER_TO_ENCODER_FACTOR = (DRIVE_FOOT_TO_ENDODER_FACTOR * 3.28084);
    double DRIVE_ENCODER_TO_METER_FACTOR = (1 / (DRIVE_METER_TO_ENCODER_FACTOR));
};

struct PreferencesTurnMotor
{
    PID_t PID_TURN;
    PID_t PID_NEWTURN;

    units::current::ampere_t MAX_AMPERAGE = 30_A;
    double TURN_RADIAN_TO_ENCODER_FACTOR = 2.03362658302;
    units::radian_t ABS_ENCODER_OFFSET = 0_rad;
};

struct PreferencesSwerve
{
    PreferencesDriveMotor DRIVE_MOTOR;
    PreferencesTurnMotor TURN_MOTOR;
    units::second_t DRIVE_RAMP_TIME = 0.3_s;
    PreferencesSwerve()
    {
        DRIVE_MOTOR.PID.Kp = 0.00001;
        DRIVE_MOTOR.PID.Kff = 0.000187;

        TURN_MOTOR.PID_TURN.Kp = 0.1;

        TURN_MOTOR.PID_NEWTURN.Kp = 0.0001;
        TURN_MOTOR.PID_NEWTURN.Kd = 10;
    }
};
static PreferencesSwerve PREFERENCE_SWERVE;

struct PreferencesArm
{
    PID_t PID;
    double TARGET_ANGLE_THRESHOLD = 5;
    double PRESET_ANGLE_THRESHOLD = 15;
    PID_t AMP_PID; // Normal PID will not work when at the AMP position, use this to configure PID for AMP
    units::degree_t ENCODER_OFFSET = 116.28_deg;
    units::degree_t MAX_LEGAL_LIMIT = 76_deg;
    units::degree_t MIN_LEGAL_LIMIT = 55_deg;
    PreferencesArm()
    {
        PID.Kp = 0.03;
        PID.Ki = 0.002;
        PID.MaxVel = 90_deg_per_s;
        PID.MaxAccel = 90_deg_per_s_sq;
        AMP_PID.Kp = 0.01;
    }

};
static PreferencesArm PREFERENCE_ARM;

struct PreferencesShamptake
{
    PID_t PID_LEFT;
    PID_t PID_RIGHT;
    double VELOCITY_NOISE = 5;
    PreferencesShamptake()
    {
        PID_LEFT.Kp = 0.0004;
        PID_LEFT.Kff = 0.000170;

        PID_RIGHT.Kp = 0.0004;
        PID_RIGHT.Kff = 0.000170;
    }
};
static PreferencesShamptake PREFERENCE_SHAMPTAKE;