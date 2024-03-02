#include <Drive/SwerveModule.h>
#include <frc/smartdashboard/SmartDashboard.h>

SwerveModule::SwerveModule(int driveID, int turningID, int canCoderID, units::degree_t offset)
: driveMotor(driveID,rev::CANSparkMax::MotorType::kBrushless),
  driveEncoder(driveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
  drivePIDController(driveMotor.GetPIDController()),
  turningMotor(turningID, rev::CANSparkMax::MotorType::kBrushless), 
  turningEncoder(turningMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
  turningPIDController(turningMotor.GetPIDController()),
  turningAbsEncoder(canCoderID), 
  absEncoderOffset(offset) {
    
    configureMotors();

    // --- CANCoder configuration ---

    ctre::phoenix6::configs::CANcoderConfiguration config;
    config.MagnetSensor.WithAbsoluteSensorRange(ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf);

    turningAbsEncoder.GetConfigurator().Apply(config);
}

SwerveModule::~SwerveModule() = default;

void SwerveModule::configureMotors() {
    // --- Drive motror config ---

    driveMotor.RestoreFactoryDefaults();

    // Set the idle mode to coast.
    driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    // Amperage limiting.
    driveMotor.SetSmartCurrentLimit(PREFERENCE_SWERVE.DRIVE_MOTOR.MAX_AMPERAGE.value());

    driveMotor.SetInverted(false);

    // Ramping.
    driveMotor.SetOpenLoopRampRate(PREFERENCE_SWERVE.DRIVE_RAMP_TIME.value());
    driveMotor.SetClosedLoopRampRate(PREFERENCE_SWERVE.DRIVE_RAMP_TIME.value());

    // PID Values.
    drivePIDController.SetP(PREFERENCE_SWERVE.DRIVE_MOTOR.PID.Kp, 0);
    drivePIDController.SetI(PREFERENCE_SWERVE.DRIVE_MOTOR.PID.Ki, 0);
    drivePIDController.SetD(PREFERENCE_SWERVE.DRIVE_MOTOR.PID.Kd, 0);
    drivePIDController.SetIZone(PREFERENCE_SWERVE.DRIVE_MOTOR.PID.Kizone, 0);
    drivePIDController.SetFF(PREFERENCE_SWERVE.DRIVE_MOTOR.PID.Kff, 0);

    // --- Turning motor config ---

    turningMotor.RestoreFactoryDefaults();

    // Coast when idle so that people can turn the module.
    turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Amperage limiting.
    turningMotor.SetSmartCurrentLimit(PREFERENCE_SWERVE.TURN_MOTOR.MAX_AMPERAGE.value());

    // It is not inverted!
    turningMotor.SetInverted(false);

    // PID Values.
    turningPIDController.SetP(PREFERENCE_SWERVE.TURN_MOTOR.PID_TURN.Kp);
    turningPIDController.SetI(PREFERENCE_SWERVE.TURN_MOTOR.PID_TURN.Ki);
    turningPIDController.SetD(PREFERENCE_SWERVE.TURN_MOTOR.PID_TURN.Kd);
    turningPIDController.SetIZone(PREFERENCE_SWERVE.TURN_MOTOR.PID_TURN.Kizone);
    turningPIDController.SetFF(PREFERENCE_SWERVE.TURN_MOTOR.PID_TURN.Kff);
    
}

void SwerveModule::doPersistentConfiguration() {
    configureMotors();

    // Burn the current configuration into the motor controllers' flash memory.
    driveMotor.BurnFlash();
    turningMotor.BurnFlash();
}

void SwerveModule::stop() {
    turningMotor.Set(0.0);
    driveMotor.Set(0.0);

    resetDrivePosition();
}

void SwerveModule::setState(frc::SwerveModuleState targetState) {
    frc::SwerveModuleState currentState = getState();

    frc::SwerveModuleState optimizedState;

    // Turn off optimization in crater mode to help with configuration.
    if (settings.isCraterMode) {
        optimizedState = targetState;
    }
    else {
        /**
         * Optimize the target state by flipping motor directions and adjusting
         * rotations in order to turn the least amount of distance possible.
         */
        optimizedState = frc::SwerveModuleState::Optimize(targetState, currentState.angle);
    }

    /**
     * Only handle turning when the robot is actually driving (Stops the modules
     * from snapping back to 0 when the robot comes to a stop).
     */
    if(units::math::abs(optimizedState.speed) > 0.01_mps) {
        // Rotate the swerve module to the desired angle.
        setTurningMotor(optimizedState.angle.Radians());
    }
  
    // Set the drive motor's velocity.
    setDriveMotor(optimizedState.speed);
}

frc::SwerveModuleState SwerveModule::getState() {
    // The velocity and rotation of the swerve module.
    return { getDriveVelocity(), getAbsoluteRotation() };
}

frc::SwerveModulePosition SwerveModule::getPosition() {
    // The position and rotation of the swerve module.
    return { getDrivePosition(), getAbsoluteRotation() };
}

void SwerveModule::resetDrivePosition() {
    driveEncoder.SetPosition(0.0);
}

units::radian_t SwerveModule::getRawRotation() {
    return turningAbsEncoder.GetAbsolutePosition().GetValue();
}

void SwerveModule::setTurningMotor(units::radian_t angle) {
    // Subtract the absolute rotation from the target rotation to get the angle to turn.
    units::radian_t angleDelta(angle - getAbsoluteRotation().Radians());
    
    /**
     * Fix the discontinuity problem by converting a -2π to 2π value into -π to π value.
     * If the value is above π rad or below -π rad...
     */
    if(units::math::abs(angleDelta).value() > std::numbers::pi) {
        const int sign = std::signbit(angleDelta.value()) ? -1 : 1;
        
        // Subtract 2π rad, or add 2π rad depending on the sign.
        angleDelta = units::radian_t(angleDelta.value() - (2 * std::numbers::pi) * sign);
    }
    
    // Convert the angle (radians) to a NEO encoder value.
    double output = angleDelta.value() * PREFERENCE_SWERVE.TURN_MOTOR.TURN_RADIAN_TO_ENCODER_FACTOR;
    
    // Add the current relative rotation to get the position to reference.
    output += getRelativeRotation();

    // Set the PID reference to the desired position.
    turningPIDController.SetReference(output, rev::CANSparkMax::ControlType::kPosition);
}

void SwerveModule::setIdleMode(rev::CANSparkMax::IdleMode idleMode) {
    driveMotor.SetIdleMode(idleMode);
}

void SwerveModule::setDriveMotor(units::meters_per_second_t velocity) {
    // Convert the velocity value (meters per second) into RPM.
    const double rpm = velocity.value() * 60 * PREFERENCE_SWERVE.DRIVE_MOTOR.DRIVE_METER_TO_ENCODER_FACTOR;

    // Set the PID reference to the desired RPM.
    drivePIDController.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
}

double SwerveModule::getRelativeRotation() {
    return turningEncoder.GetPosition();
}

frc::Rotation2d SwerveModule::getAbsoluteRotation() {
    // The angle from the CANCoder.
    units::degree_t angle(getRawRotation());

    // Subtract the offset from the angle.
    angle -= absEncoderOffset;

    return angle;
}

double SwerveModule::getRawDriveEncoder() {
    return driveEncoder.GetPosition();
}

units::meters_per_second_t SwerveModule::getDriveVelocity() {
    // Convert the RPM to a velocity value (meters per second).
    const double mps = (driveEncoder.GetVelocity() / 60) * PREFERENCE_SWERVE.DRIVE_MOTOR.DRIVE_ENCODER_TO_METER_FACTOR;
    
    return units::meters_per_second_t(mps);
}

units::meter_t SwerveModule::getDrivePosition() {
    // Convert the rotations to meters.
    const double m = driveEncoder.GetPosition() * PREFERENCE_SWERVE.DRIVE_MOTOR.DRIVE_ENCODER_TO_METER_FACTOR;

    return units::meter_t(m);
}

void SwerveModule::sendFeedback(std::size_t moduleIndex) {
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_RawRotation_deg",    moduleIndex), units::degree_t(getRawRotation()).value());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_Rotation_deg",       moduleIndex), getAbsoluteRotation().Degrees().value());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_EncoderOffset_deg",  moduleIndex), units::degree_t(absEncoderOffset).value());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_EncoderOffset_rad",  moduleIndex), units::radian_t(absEncoderOffset).value());

    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_EncoderRotation",    moduleIndex), getRelativeRotation());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_EncoderDrive",       moduleIndex), getRawDriveEncoder());

    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_Velocity_mps",       moduleIndex), getDriveVelocity().value());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_TempTurning_C",      moduleIndex), turningMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_TempDrive_C",        moduleIndex), driveMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_CurrentDrive_A",     moduleIndex), driveMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber(fmt::format("Module_{}_CurrentTurning_A",   moduleIndex), turningMotor.GetOutputCurrent());

    // hi jeff
}