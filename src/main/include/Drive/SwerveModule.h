#pragma once

#include <Basic/IOMap.h>
#include <Basic/Mechanism.h>
#include <Util/Preferences.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/current.h>
#include <numbers>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>

#include <frc/controller/PIDController.h>

class SwerveModule : public Mechanism {
public:
    SwerveModule(int driveID, int turningID, int canCoderID, units::degree_t offset);
    ~SwerveModule();

    void sendFeedback(std::size_t moduleIndex);
    void doPersistentConfiguration() override;

    /**
     * Stop all motors.
     */
    void stop();

    /**
     * Sets the state of the swerve module (Velocity and angle).
     */
    void setState(frc::SwerveModuleState state);

    /**
     * Returns the current state of the swerve module (Velocity and angle).
     */
    frc::SwerveModuleState getState();

    /**
     * Returns the current position of the swerve module (Position and angle).
     */
    frc::SwerveModulePosition getPosition();

    /**
     * Resets the drive encoder positions.
     */
    void resetDrivePosition();

    /**
     * Returns the raw rotation of the absolute turning encoder (Without
     * offsets applied).
     */
    units::radian_t getRawRotation();

    /**
     * Sets the angle of the swerve module.
     */
    void setTurningMotor(units::radian_t angle);

    /**
     * Sets the idle mode of the drive motor controller.
     */
    void setIdleMode(rev::CANSparkMax::IdleMode idleMode);

private:
    /**
     * Sets the velocity of the drive motor.
     */
    void setDriveMotor(units::meters_per_second_t velocity);
    
    /**
     * Configure motors to power-on states
     */
    void configureMotors();

    /**
     * Returns the relative rotation of the module (Rotations of the NEO 550).
     */
    double getRelativeRotation();

    /**
     * Returns the absolute rotation of the module (CANCoder encoder value
     * with offsets applied).
     */
    frc::Rotation2d getAbsoluteRotation();

    /**
     * Returns the raw value of the drive motor's encoder (rotations).
     */
    double getRawDriveEncoder();
    
    /**
     * Returns the current velocity of the drive motor (meters per second).
     */
    units::meters_per_second_t getDriveVelocity();

    /**
     * Returns the current position of the drive motor (meters).
     */
    units::meter_t getDrivePosition();

    // The drive motor.
    rev::CANSparkMax driveMotor;
    rev::SparkRelativeEncoder driveEncoder;
    rev::SparkPIDController drivePIDController;
    // The turning motor.
    rev::CANSparkMax turningMotor;
    rev::SparkRelativeEncoder turningEncoder;
    rev::SparkPIDController turningPIDController;

    // The absolute encoder.
    ctre::phoenix6::hardware::CANcoder turningAbsEncoder;

    // The offset of the turning absolute encoder.
    units::radian_t absEncoderOffset = PREFERENCE_SWERVE.TURN_MOTOR.ABS_ENCODER_OFFSET;
};