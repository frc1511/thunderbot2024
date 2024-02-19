#include <Drive/Drive.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

Drive::Drive() {
    manualThetaPIDController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));
}

Drive::~Drive() {
    for (SwerveModule* module : swerveModules) {
       delete module;
    }
}

void Drive::doPersistentConfiguration() {
    for (SwerveModule* module : swerveModules) {
        module->doPersistentConfiguration();
    }
}

void Drive::resetToMode(MatchMode mode) {
    resetPIDControllers();

    driveMode = DriveMode::STOPPED;

    // Reset the manual control data.
    VelocityControlData lastControlData(controlData);
    controlData = { 0_mps, 0_mps, 0_rad_per_s, ControlFlag::NONE };

    // This seems to be necessary. Don't ask me why.
    for (SwerveModule* module : swerveModules) {
        module->stop();
    }

    if (mode == MatchMode::DISABLED) {
        /**
         * Coast all motors in disabled (good for transportation, however can
         * lead to some runaway robots).
         */
        setIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
    else {
        // Brake all motors when enabled to help counteract pushing.
        setIdleMode(rev::CANSparkMax::IdleMode::kBrake);

        /**
         * Calibrate the IMU if not already calibrated. This will cause the
         * robot to pause for 4 seconds while it waits for it to calibrate, so
         * the IMU should always be calibrated before the match begins.
         */
        if (!isIMUCalibrated()) {
            calibrateIMU();
        }
    }

    static bool wasAuto = false;

    // Going from Auto to Disabled to Teleop.
    if (wasAuto && mode == Mechanism::MatchMode::TELEOP) {
        wasAuto = false;
        frc::Pose2d currPose(getEstimatedPose());
        resetOdometry(frc::Pose2d(currPose.X(), currPose.Y(), (180_deg - currPose.Rotation().Degrees()) + 90_deg + (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg)));
    }
    else {
        // Check if going from Auto to Disabled.
        wasAuto = getLastMode() == Mechanism::MatchMode::AUTO && mode == Mechanism::MatchMode::DISABLED;

        // Doing something else.
        if (!wasAuto && mode != Mechanism::MatchMode::DISABLED) {
            // Stuff to reset normally.
        }
    }
}


void Drive::process() {
    updateOdometry();

    switch (driveMode) {
        case DriveMode::STOPPED:
            execStopped();
            break;
        case DriveMode::VELOCITY:
            execVelocityControl();
            break;
        case DriveMode::TRAJECTORY:
            if (trajectoryFinished()) {
                driveMode = DriveMode::TRAJECTORY_FINISHED;
                break;
            }
            // Execute the follow trajectory command.
            exeFollowTrajectory();
            break;
        case DriveMode::TRAJECTORY_FINISHED:
            execStopped();
            break;
    }
}

void Drive::manualControlRelRotation(double xPct, double yPct, double angPct, unsigned flags) {
    /**
     * Calculate chassis velocities using percentages of the configured max
     * manual control velocities.
     */
    units::meters_per_second_t xVel    = xPct * DRIVE_MANUAL_MAX_VEL;
    units::meters_per_second_t yVel    = yPct * DRIVE_MANUAL_MAX_VEL;
    units::radians_per_second_t angVel = angPct * DRIVE_MANUAL_MAX_ANG_VEL;

    // Pass the velocities to the velocity control function.
    velocityControlRelRotation(xVel, yVel, angVel, flags);
}

void Drive::manualControlAbsRotation(double xPct, double yPct, units::radian_t angle, unsigned flags) {
    /**
     * Calculate chassis velocities using percentages of the configured max
     * manual control velocities.
     */
    units::meters_per_second_t xVel    = xPct * DRIVE_MANUAL_MAX_VEL;
    units::meters_per_second_t yVel    = yPct * DRIVE_MANUAL_MAX_VEL;

    // Pass the velocities to the velocity control function.
    velocityControlAbsRotation(xVel, yVel, angle, flags);
}

void Drive::exeFollowTrajectory() {
    // Calculate chassis velocities that are required in order to reach the
    // desired state.
    frc::ChassisSpeeds targetChassisSpeeds = trajectoryController.getVelocities(getEstimatedPose());

    // Drive!
    setModuleStates(targetChassisSpeeds);

    //hi ishan
    //hi jeff
    //hi trevor
    //hi nevin
    //hi josh
    //hi byers
    //hi calla
    //hi nadia
    //hi homer 2.0
    //hi charlie
    //hi yaqoub
    //hi mason
    //hi ben
}

bool Drive::trajectoryFinished() {
    if (driveMode == DriveMode::TRAJECTORY) {
        if (trajectoryController.atReference(getEstimatedPose())) {
            return true;
        }
        return false;
    }
    return true;
}

void Drive::velocityControlRelRotation(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radians_per_second_t angVel, unsigned flags) {
    units::meters_per_second_t newXVel    = xVel;
    units::meters_per_second_t newYVel    = yVel;
    units::radians_per_second_t newAngVel = angVel;
    
    // Apply the locked axis flags.
    if (flags & ControlFlag::LOCK_X) newXVel = 0_mps;
    if (flags & ControlFlag::LOCK_Y) newYVel = 0_mps;
    if (flags & ControlFlag::LOCK_ROT) newAngVel = 0_rad_per_s;

    // Stop the robot in brick mode no matter what.
    if (flags & ControlFlag::BRICK) {
        driveMode = DriveMode::STOPPED;
    }
    // The robot isn't being told to do move, sooo.... stop??
    else if ((!newXVel && !newYVel && !newAngVel)) {
        driveMode = DriveMode::STOPPED;
    }
    // The robot is being told to do stuff, so start doing stuff.
    else {
        driveMode = DriveMode::VELOCITY;
    }

    controlData = { newXVel, newYVel, newAngVel, flags };
}

void Drive::velocityControlAbsRotation(units::meters_per_second_t xVel, units::meters_per_second_t yVel, units::radian_t angle, unsigned flags) {
    units::radian_t currAngle = getEstimatedPose().Rotation().Radians();

    // Reset the PID controller if this is the first run.
    static bool firstRun = true;
    if (firstRun) {
        manualThetaPIDController.Reset(currAngle);
        firstRun = false;
    }

    auto angVel = units::radians_per_second_t(
        manualThetaPIDController.Calculate(currAngle, angle)
    );

    velocityControlRelRotation(xVel, yVel, angVel, flags);
}

void Drive::calibrateIMU() {
    pigeon.Reset();
    
    imuCalibrated = true;

    resetOdometry();
}

bool Drive::isIMUCalibrated() {
    return imuCalibrated;
}

void Drive::resetOdometry(frc::Pose2d pose) {
    /**
     * Resets the position and rotation of the robot to a given pose
     * while ofsetting for the IMU's recorded rotation.
     */

    //poseEstimator.ResetPosition(getRotation(), getModulePositions(), pose);

    pigeon.SetYaw(0_deg);

    for (SwerveModule* module : swerveModules) {
        module->resetDrivePosition();
    }
}

frc::Pose2d Drive::getEstimatedPose() {
    return frc::Pose2d(0_m, 0_m, 0_deg);
    //return poseEstimator.GetEstimatedPosition();
}

frc::Rotation2d Drive::getRotation() {
    // The raw rotation from the IMU.
    return frc::Rotation2d(pigeon.GetYaw().GetValue());
}

void Drive::resetPIDControllers() {

    frc::Pose2d currPose(getEstimatedPose());
    units::degree_t rotation(currPose.Rotation().Degrees());

    manualThetaPIDController.Reset(rotation);
}

void Drive::cmdDriveToPose(units::meter_t x, units::meter_t y, frc::Rotation2d angle, YaqoubsTrajectoryConfig config) {
    driveMode = DriveMode::TRAJECTORY;

    trajectoryController.setTrajectory(getEstimatedPose(), { x, y, units::math::fmod(angle.Degrees(), 360_deg) }, config);
}

void Drive::updateOdometry() {
    /**
     * Update the pose estimator with encoder measurements from
     * the swerve modules.
     */
    //poseEstimator.Update(getRotation(), getModulePositions());
}

void Drive::execStopped() {
    // Set the speeds to 0.
    setModuleStates({ 0_mps, 0_mps, 0_deg_per_s });

    // Just for feedback.
    targetPose = getEstimatedPose();

    // Put the drivetrain into brick mode if the flag is set.
    if (controlData.flags & ControlFlag::BRICK) {
        makeBrick();
    }

}

void Drive::execVelocityControl() {
    frc::Pose2d currPose(getEstimatedPose());
    
    frc::ChassisSpeeds velocities;

    // Generate chassis speeds depending on the control mode.
    if (controlData.flags & ControlFlag::FIELD_CENTRIC) {
        // Generate chassis speeds based on the rotation of the robot relative to the field.
        velocities = frc::ChassisSpeeds::FromFieldRelativeSpeeds(controlData.xVel, controlData.yVel, controlData.angVel, currPose.Rotation());//whooshWhoosh->getHeadingAngle());// currPose.Rotation());
    }
    else {
        // Chassis speeds are robot-centric.
        velocities = { controlData.xVel, controlData.yVel, controlData.angVel };
    }

    // Just for feedback.
    targetPose = currPose;

    // Set the modules to drive at the given velocities.
    setModuleStates(velocities);

}

void Drive::makeBrick() {
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        units::degree_t angle;
        // If the index is even.
        if (i % 2 == 0) {
            angle = -45_deg;
        }
        // If the index is odd.
        else {
            angle = 45_deg;
        }
        
        // Stop the robot. It should already be stopped tho.
        driveMode = DriveMode::STOPPED;
        
        // Turn the swerve module to point towards the center of the robot.
        swerveModules.at(i)->setTurningMotor(angle);
    }
}

void Drive::setIdleMode(rev::CANSparkMax::IdleMode mode) {
    for (SwerveModule* module : swerveModules) {
        module->setIdleMode(mode);
    }
}

void Drive::setModuleStates(frc::ChassisSpeeds speeds) {
    // Store velocities for feedback.
    chassisSpeeds = speeds;

    // Generate individual module states using the chassis velocities.
    wpi::array<frc::SwerveModuleState, 4> moduleStates(kinematics.ToSwerveModuleStates(speeds));
    
    // Set the states of the individual modules.
    for(std::size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules.at(i)->setState(moduleStates.at(i));
    }
}

wpi::array<frc::SwerveModuleState, 4> Drive::getModuleStates() {
    return { swerveModules.at(0)->getState(), swerveModules.at(1)->getState(),
             swerveModules.at(2)->getState(), swerveModules.at(3)->getState() };
}

wpi::array<frc::SwerveModulePosition, 4> Drive::getModulePositions() {
    return { swerveModules.at(0)->getPosition(), swerveModules.at(1)->getPosition(),
             swerveModules.at(2)->getPosition(), swerveModules.at(3)->getPosition() };
}

void Drive::sendFeedback() {
    // Module feedback.
    for (std::size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules.at(i)->sendFeedback(i);
    }

    frc::Pose2d pose(getEstimatedPose());

    // Drive feedback.
    frc::SmartDashboard::PutNumber("Drive_PoseY_m",                 pose.X().value());
    frc::SmartDashboard::PutNumber("Drive_PoseX_m",                 pose.Y().value());
    frc::SmartDashboard::PutNumber("Drive_PoseRot_deg",             getRotation().Degrees().value());
    frc::SmartDashboard::PutNumber("Drive_ControlVelX_mps",         controlData.xVel.value());
    frc::SmartDashboard::PutNumber("Drive_ControlVelY_mps",         controlData.yVel.value());
    frc::SmartDashboard::PutNumber("Drive_ControlVelRot_rad_per_s", controlData.angVel.value());
    frc::SmartDashboard::PutBoolean("Drive_FieldCentric",           controlData.flags & ControlFlag::FIELD_CENTRIC);
    frc::SmartDashboard::PutBoolean("Drive_Brick",                  controlData.flags & ControlFlag::BRICK);
    frc::SmartDashboard::PutBoolean("Drive_LockX",                  controlData.flags & ControlFlag::LOCK_X);
    frc::SmartDashboard::PutBoolean("Drive_LockY",                  controlData.flags & ControlFlag::LOCK_Y);
    frc::SmartDashboard::PutBoolean("Drive_LockRot",                controlData.flags & ControlFlag::LOCK_ROT);

    // ThunderDashboard things.
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_x_pos",        pose.X().value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_y_pos",        pose.Y().value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_target_x_pos", targetPose.X().value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_target_y_pos", targetPose.Y().value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_x_vel",        chassisSpeeds.vx.value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_y_vel",        chassisSpeeds.vy.value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_ang_vel",      chassisSpeeds.omega.value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_ang",          pose.Rotation().Radians().value());
    frc::SmartDashboard::PutNumber("thunderdashboard_drive_target_ang",   targetPose.Rotation().Radians().value());

    frc::SmartDashboard::PutBoolean("thunderdashboard_gyro", !imuCalibrated);
}