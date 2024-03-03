#include <GamEpiece/Shamptake.h>

Shamptake::Shamptake()
: shooterMotorRightPIDController(shooterMotorRight.GetPIDController()),
  shooterMotorRightEncoder(shooterMotorRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
  shooterMotorLeftPIDController(shooterMotorLeft.GetPIDController()),
  shooterMotorLeftEncoder(shooterMotorLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)) {
    shooterMotorRight.SetInverted(false);
    shooterMotorLeft.SetInverted(true);

    shooterMotorRightPIDController.SetP(PREFERENCE_SHAMPTAKE.PID_RIGHT.Kp);
    shooterMotorRightPIDController.SetI(PREFERENCE_SHAMPTAKE.PID_RIGHT.Ki);
    shooterMotorRightPIDController.SetD(PREFERENCE_SHAMPTAKE.PID_RIGHT.Kd);
    shooterMotorRightPIDController.SetIZone(PREFERENCE_SHAMPTAKE.PID_RIGHT.Kizone);
    shooterMotorRightPIDController.SetFF(PREFERENCE_SHAMPTAKE.PID_RIGHT.Kff);
    shooterMotorRightPIDController.SetOutputRange(0, 1);

    shooterMotorLeftPIDController.SetP(PREFERENCE_SHAMPTAKE.PID_LEFT.Kp);
    shooterMotorLeftPIDController.SetI(PREFERENCE_SHAMPTAKE.PID_LEFT.Ki);
    shooterMotorLeftPIDController.SetD(PREFERENCE_SHAMPTAKE.PID_LEFT.Kd);
    shooterMotorLeftPIDController.SetIZone(PREFERENCE_SHAMPTAKE.PID_LEFT.Kizone);
    shooterMotorLeftPIDController.SetFF(PREFERENCE_SHAMPTAKE.PID_LEFT.Kff);
    shooterMotorLeftPIDController.SetOutputRange(0, 1);
    stop();
}

Shamptake::~Shamptake() {
    
}

void Shamptake::sendFeedback() {
    frc::SmartDashboard::PutString("Shamptake_intakeMode", intakeModeString());
    frc::SmartDashboard::PutNumber("Shamptake_shooterLeftRPM", shooterMotorLeftEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shamptake_shooterRightRPM", shooterMotorRightEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shamptake_shooterTargetRPM", targetShooterRPM);
    frc::SmartDashboard::PutBoolean("Shamptake_shooterAtRPM", atTargetRPM());
}

bool Shamptake::isNoteSensorTripped()
{
    return !noteSensor.Get();
}

void Shamptake::doPersistentConfiguration() {
}

void Shamptake::resetToMode(MatchMode mode) {
    sensorDetected = false;
    trippedBefore = false;
    isAuto = (mode == MatchMode::AUTO);
    stop();
}

bool Shamptake::atTargetRPM() {
    bool isAtTarget = (shooterMotorRightEncoder.GetVelocity() >= targetShooterRPM &&
                       shooterMotorLeftEncoder.GetVelocity() >= targetShooterRPM);
    return isAtTarget;
}

bool Shamptake::notShooting() {
    return !autoShooting;
}

bool Shamptake::notIntaking() {
    return !autoIntaking;
}

bool Shamptake::hasGamepiece() {
    return trippedBefore;
}


void Shamptake::process() {
    //if no snesor detected
        //run intake (Before Sensor) (IntakeSpeed = Normal)
        //except if intake previously tripped then stop (Now past sensor) (IntakeSpeed = Stop)
            //but if shooting, intake max (Now leaving) (IntakeSpeed = Shooting) (Handled in controls)
    //if sensor detected (In Sensor range)
        //slow intake (IntakeSpeed = Slow)
        //intake previously tripped = true (So we stop when past)
    //if outtaking
        //Outtake (Handled in controls)
        //intake previously tripped = false (So we don't stop when inkaing next time)
    sensorDetected = isNoteSensorTripped();
    
    if (!sensorDetected) {
        if (trippedBefore) { // Past Sensor
            //sleep(0.7);
            intakeSpeed = IntakeSpeed::STOP_INTAKE;
            if (autoIntaking) {
                autoIntaking = false;
                stopIntake();
            }
        } else { // Before Sensor
            intakeSpeed = IntakeSpeed::NORMAL_INTAKE;
        }
    } else { // Sensor Tripped
        intakeSpeed = IntakeSpeed::SLOW_INTAKE;
        trippedBefore = true;
        hasNote = true;
    }

    if (autoShooting) {
        if (atTargetRPM() && notIntaking()) {
            intakeSpeed = IntakeSpeed::FIRE_INTAKE;
            shooterTimer.Start();
            autoIntaking = true;
        }
        if (shooterTimer.Get() >= 1_s) {
            shooterTimer.Stop();
            stop();
        }
    }
    if (!autoIntaking && isAuto) {
        intakeSpeed = IntakeSpeed::STOP_INTAKE;
    }
}

std::string Shamptake::intakeModeString() {
    std::string modeString = "ERROR";
    switch (intakeSpeed)
    {
    case IntakeSpeed::NORMAL_INTAKE:
        modeString = "NORMAL";
        break;
    case IntakeSpeed::STOP_INTAKE:
        modeString = "STOP";
        break;
    case IntakeSpeed::SLOW_INTAKE:
        modeString = "SLOW";
        break;
    case IntakeSpeed::FIRE_INTAKE:
        modeString = "FIRE";
        break;
    case IntakeSpeed::OUTTAKE_INTAKE:
        modeString = "OUTTAKE";
        break;
    default:
        break;
    }
    return modeString;
}

void Shamptake::runMotors() {
    double iSpeed = presetIntakeSpeeds[intakeSpeed];
    intake(iSpeed);

    double sSpeed = presetShooterSpeeds[shooterSpeed];
    shooter(sSpeed);
}

void Shamptake::intake(double power) {
    intakeMotor1.Set(power);
}

void Shamptake::shooter(double RPM) {
    targetShooterRPM = RPM;
    shooterMotorLeftPIDController.SetReference(targetShooterRPM, rev::CANSparkBase::ControlType::kVelocity);
    shooterMotorRightPIDController.SetReference(targetShooterRPM, rev::CANSparkBase::ControlType::kVelocity);
}

void Shamptake::autoIntake() {
    autoIntaking = true;
    intakeSpeed = IntakeSpeed::NORMAL_INTAKE;
}

void Shamptake::autoShoot() {
    autoShooting = true;
    shooterTimer.Reset();
    shooterSpeed = ShooterSpeed::AUTO_FIRE_SHOOTER;
}

void Shamptake::stopIntake() {
    intakeSpeed = IntakeSpeed::STOP_INTAKE;
    autoIntaking = false;
}

void Shamptake::stopShooter() {
    shooterSpeed = ShooterSpeed::STOP_SHOOTER;
    autoShooting = false;
}

void Shamptake::stop() {
    stopIntake();
    stopShooter();
}