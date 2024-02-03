#pragma once

#include <Basic/Mechanism.h>
#include <Basic/IOMap.h>
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

struct YaqoubsTrajectoryConfig {
    units::meters_per_second_t maxVelocity = maxVelocity;
    units::meters_per_second_squared_t maxAcceleration = maxAcceleration;
    units::radians_per_second_t maxAngularVelocity = maxAngularVelocity;
    units::radians_per_second_t minAngularVelocity = minAngularVelocity;
    double angularVelocityFactor = angularVelocityFactor;
    units::meters_per_second_t startVelocity = 0_mps;
    units::meters_per_second_t endVelocity = 0_mps;
};

/**
 * Represents a trajectory controller for the robot to utilize when following
 * trajectories. I am not using the frc::Trajectory class because it doesn't
 * work right and everything is very sad when we use it. Nuf said. :D
 */
class YaqoubsTrajectoryController {
public:
    YaqoubsTrajectoryController();
    ~YaqoubsTrajectoryController();

    void sendFeedback();

    /**
     * Sets the trajectory for the controller to reference.
     */
    void setTrajectory(frc::Pose2d currentPose, frc::Pose2d endPose, YaqoubsTrajectoryConfig config = {});
    
    /**
     * Returns whether the robot is at the final state of the trajectory.
     */
    bool atReference(frc::Pose2d currentPose);

    /**
     * Returns the velocities of the chassis that are required to drive the
     * robot along the reference trajectory.
     */
    frc::ChassisSpeeds getVelocities(frc::Pose2d currentPose);
    
private:
    enum DriveState {
        UNKNOWN,
        ACCELERATING,
        CONSTANT,
        DECELERATING,
    };
    
    // The state of the controller.
    DriveState driveState = DriveState::UNKNOWN;

    // Timer for acceleration and deceleration.
    frc::Timer timer {};
    
    // The start position and rotation.
    frc::Pose2d start {};
    // The desired end position and rotation.
    frc::Pose2d end {};

    // The maximum velocity to reach.
    units::meters_per_second_t maxVelocity = 0_mps;
    // The maximum acceleration.
    units::meters_per_second_squared_t maxAcceleration = 0_mps_sq;
    // The maximum angular velocity.
    units::radians_per_second_t maxAngularVelocity = 0_rad_per_s;
    // The minimum angular velocity.
    units::radians_per_second_t minAngularVelocity = 0_rad_per_s;
    // The angular velocity acceleration and deceleration factor.
    double angularVelocityFactor = 0;
    // The starting velocity.
    units::meters_per_second_t startVelocity = 0_mps;
    // The ending velocity.
    units::meters_per_second_t endVelocity = 0_mps;

    // The distance that has been traveled.
    units::meter_t distanceTraveled = 0_m;
    
    // The distance to be traveled.
    units::meter_t totalDistance = 0_m;
    // The angle to drive in.
    units::degree_t heading = 0_deg;
    
    // The distance to accelerate.
    units::meter_t accelerateDistance = 0_m;
    // The distance to decelerate.
    units::meter_t decelerateDistance = 0_m;
    // The distance at maximum velocity.
    units::meter_t constantDistance = 0_m;

    // Whether the robot has successfully driven to the target position.
    bool driveFinished = false;
    // Whether the robot has successfully rotated to the target angle.
    bool rotateFinished = false;
};