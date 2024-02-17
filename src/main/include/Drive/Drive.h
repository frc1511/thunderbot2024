#pragma once

#include <Basic/Mechanism.h>
#include <Basic/IOMap.h>
#include <Drive/Trajectory.h>
#include <Drive/SwerveModule.h>
#include <ctre/phoenix6/Pigeon2.hpp>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/Timer.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/math.h>
#include <wpi/array.h>
#include <memory>
#include <numbers>
#include <fstream>
#include <map>
#include <algorithm>

#define ROBOT_WIDTH 20.1875_in // Distences from center of each swerve module
#define ROBOT_LENGTH 20.1875_in 

#define DRIVE_MANUAL_MAX_VEL       3.8_mps
#define DRIVE_MANUAL_MAX_ANG_VEL   360_deg_per_s
#define DRIVE_MANUAL_MAX_ANG_ACCEL 6.28_rad_per_s_sq
// Drivetrain X and Y PID values.
#define DRIVE_XY_P 3.25
#define DRIVE_XY_I 0.0
#define DRIVE_XY_D 0.1

// Drivetrain Theta PID values.
#define DRIVE_THETA_P 8.0
#define DRIVE_THETA_I 0.0
#define DRIVE_THETA_D 0.1

class Drive : public Mechanism {
public:
    Drive();
    ~Drive();

    void process() override;
    void sendFeedback() override;
    void doPersistentConfiguration() override;
    void resetToMode(MatchMode mode) override;

    /**
     * A number of flags that specify different control features
     * of the robot during manual control.
     */
    enum ControlFlag {
        NONE          = 0,
        FIELD_CENTRIC = 1 << 0, // Field-relative control (forward is always field-forward).
        BRICK         = 1 << 1, // All modules pointed towards the center.
        LOCK_X        = 1 << 3, // Lock X-axis drivetrain movement.
        LOCK_Y        = 1 << 4, // Lock Y-axis drivetrain movement.
        LOCK_ROT      = 1 << 5, // Lock rotation drivetrain movement.
    };

    /**
     * Controls the speeds of drivetrain using percentages of the max speed.
     * (The direction of the velocities is dependant on the control type).
     * 
     * Positive xPct   -> Move right,             Negative xPct   -> Move left.
     * Positive yPct   -> Move forward,           Negative yPct   -> Move backward.
     * Positive angPct -> Turn counter-clockwise, Negative angPct -> Turn clockwise.
     * 
     * Control flags are used to control the behavior of the drivetrain.
     */
    void manualControlRelRotation(double xPct, double yPct, double angPct, unsigned flags);


    /**
     * Controls the speeds of the drivetrain using percentages of the max speed
     * and the desired rotation. (The direction of the velocities is dependant on
     * the control type). The angle is based on the robot's knowledge about it's
     * position on the field.
     * 
     * Positive xPct   -> Move right,             Negative xPct   -> Move left.
     * Positive yPct   -> Move forward,           Negative yPct   -> Move backward.
    */
    void manualControlAbsRotation(double xPct, double yPct, units::radian_t angle, unsigned flags);
    /**
     * Controls the speeds of the drivetrain using the velocities specified.
     * (The direction of the velocities is dependant on the control type).
    */
    void velocityControlRelRotation(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radians_per_second_t angVel, unsigned flags);

    /**
     * Controls the speeds of the drivetrain using the velocities specified. 
     * (The direction of the velocities is dependant on the control type).
     * The angle is based on the robot's knowledge about it's position on the
     * field.
     */
    void velocityControlAbsRotation(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radian_t angle, unsigned flags);


    /**
     * Calibrates the IMU (Pauses the robot for 4 seconds while it calibrates).
     */
    void calibrateIMU();

    /**
     * Returns whether the IMU is calibrated.
     */
    bool isIMUCalibrated();

    /**
     * Resets the position and rotation of the drivetrain on the field to a
     * specified pose.
     */
    void resetOdometry(frc::Pose2d pose = frc::Pose2d());

    /**
     * Returns the position and rotation of the robot on the field.
     */
    frc::Pose2d getEstimatedPose();

    /**
     * Returns the raw rotation of the robot as recorded by the IMU.
     */
    frc::Rotation2d getRotation();

    /**
     * Resets all drive PID controllers.
     */
    void resetPIDControllers();

    /**
     * Executes instructions for when the robot is stopped.
     */
    void execStopped();

    /**
     * Move the robot a distance (in feet) at a speed 
     */
    void moveDistance(double distance, units::meters_per_second_t speed);


    enum class DriveMode {
        STOPPED,
        VELOCITY,
        TRAJECTORY,
        TRAJECTORY_FINISHED
    };
    
    /**
     * Set the drive mode
     */

    void setMode(DriveMode mode) {
        driveMode = mode;
    };
    /**
     * The helper class that it used to convert chassis speeds into swerve
     * module states.
     */
    frc::SwerveDriveKinematics<4> kinematics { locations };
    /**
     * The helper class that it used to convert swerve
     * module states into chassis speeds.
     */
    frc::SwerveDriveKinematics<4> moduleStates { kinematics };

    /**
     * The class that handles tracking the position of the robot on the field
     * during the match.
     */
    // frc::SwerveDrivePoseEstimator<4> poseEstimator {
    //     kinematics,
    //     getRotation(),
    //     getModulePositions(),
    //     frc::Pose2d(),
    //     { 0.0, 0.0, 0.0 }, // Standard deviations of model states.
    //     { 1.0, 1.0, 1.0 } // Standard deviations of the vision measurements.
    // };
    // PID Controller for angular drivetrain movement.
    frc::ProfiledPIDController<units::radians> manualThetaPIDController {
        DRIVE_THETA_P, DRIVE_THETA_I, DRIVE_THETA_D,
        frc::TrapezoidProfile<units::radians>::Constraints(DRIVE_MANUAL_MAX_ANG_VEL, DRIVE_MANUAL_MAX_ANG_ACCEL)
    };
    /**
     * Sets the velocities of the drivetrain.
     */
    void setModuleStates(frc::ChassisSpeeds speeds);

    /**
     * Begins a command to drive and rotate to a specified pose.
     */
    void cmdDriveToPose(units::meter_t x, units::meter_t y, frc::Rotation2d angle, YaqoubsTrajectoryConfig config = YaqoubsTrajectoryConfig());

private:

    /**
     * Executes the current follow trajectory command.
    */
    void exeFollowTrajectory();

    /**
     * Returns whether the trajectory has finished and sets the drive mode to TRAJECTORY_FINISHED if finished.
     */
    bool trajectoryFinished();
    /**
     * Updates the position and rotation of the drivetrain on the field.
     */
    void updateOdometry();


    /**
     * Executes the instructions for when the robot is in velocity control.
     */
    void execVelocityControl();

    /**
     * Puts the drivetrain into brick mode (all modules turned towards the
     * center).
     */
    void makeBrick();

    /**
     * Sets the idle mode of the drive motors.
     */
    void setIdleMode(rev::CANSparkMax::IdleMode mode);


    /**
     * Returns the states of the swerve modules. (velocity and rotatation)
     */
    //wpi::array<frc::SwerveModuleState, 4> getModuleStates();

    /**
     * Returns the positions of the swerve modules.
     */
    // wpi::array<frc::SwerveModulePosition, 4> getModulePositions();


    bool imuCalibrated = false;

    // The locations of the swerve modules on the robot.
    wpi::array<frc::Translation2d, 4> locations {
        frc::Translation2d(-ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front left.
        frc::Translation2d(-ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back left.
        frc::Translation2d(+ROBOT_WIDTH/2, -ROBOT_LENGTH/2), // Back right.
        frc::Translation2d(+ROBOT_WIDTH/2, +ROBOT_LENGTH/2), // Front right.
    };

    // The swerve modules on the robot.
    // wpi::array<SwerveModule*, 4> swerveModules {
    //     new SwerveModule(CAN_SWERVE_DRIVE_FL, CAN_SWERVE_ROTATION_FL, CAN_SWERVE_CANCODER_FL, -123.134766_deg+180_deg - 90_deg),
    //     new SwerveModule(CAN_SWERVE_DRIVE_BL, CAN_SWERVE_ROTATION_BL, CAN_SWERVE_CANCODER_BL, -17.666016_deg+180_deg - 90_deg),
    //     new SwerveModule(CAN_SWERVE_DRIVE_BR, CAN_SWERVE_ROTATION_BR, CAN_SWERVE_CANCODER_BR, -62.753906_deg - 90_deg),
    //     new SwerveModule(CAN_SWERVE_DRIVE_FR, CAN_SWERVE_ROTATION_FR, CAN_SWERVE_CANCODER_FR, -126.298828_deg - 90_deg)
    // };

    ctre::phoenix6::hardware::Pigeon2 pigeon { CAN_PIGEON };


    // The current drive mode.
    DriveMode driveMode = DriveMode::STOPPED;

    struct VelocityControlData {
        units::meters_per_second_t xVel;
        units::meters_per_second_t yVel;
        units::radians_per_second_t angVel;
        unsigned flags = ControlFlag::NONE;
    };

    // The data concerning velocity control.
    VelocityControlData controlData {};
    
    // The trajectory controller.
    YaqoubsTrajectoryController trajectoryController {};
    

    // Feedback variables.
    frc::ChassisSpeeds chassisSpeeds { 0_mps, 0_mps, 0_rad_per_s };
    frc::Pose2d targetPose;
};