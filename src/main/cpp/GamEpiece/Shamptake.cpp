#include <GamEpiece/Shamptake.h>

Shamptake::Shamptake()
: shooterMotorRightPIDController(shooterMotorRight.GetPIDController()),
  shooterMotorRightEncoder(shooterMotorRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
  shooterMotorLeftPIDController(shooterMotorLeft.GetPIDController()),
  shooterMotorLeftEncoder(shooterMotorLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)) {
    shooterMotorRight.SetInverted(false);
    shooterMotorLeft.SetInverted(true);

    shooterMotorRightPIDController.SetP(SHAMPTANK_RIGHT_MOTOR_P);
    shooterMotorRightPIDController.SetI(SHAMPTANK_RIGHT_MOTOR_I);
    shooterMotorRightPIDController.SetD(SHAMPTANK_RIGHT_MOTOR_D);
    shooterMotorRightPIDController.SetIZone(SHAMPTANK_RIGHT_MOTOR_I_ZONE);
    shooterMotorRightPIDController.SetFF(SHAMPTANK_RIGHT_MOTOR_FEED_FOWARD);
    shooterMotorRightPIDController.SetOutputRange(0, 1);

    shooterMotorLeftPIDController.SetP(SHAMPTANK_LEFT_MOTOR_P);
    shooterMotorLeftPIDController.SetI(SHAMPTANK_LEFT_MOTOR_I);
    shooterMotorLeftPIDController.SetD(SHAMPTANK_LEFT_MOTOR_D);
    shooterMotorLeftPIDController.SetIZone(SHAMPTANK_LEFT_MOTOR_I_ZONE);
    shooterMotorLeftPIDController.SetFF(SHAMPTANK_LEFT_MOTOR_FEED_FOWARD);
    shooterMotorLeftPIDController.SetOutputRange(0, 1);
    shooter(0);
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

void Shamptake::doPersistentConfiguration() {

}

void Shamptake::resetToMode(MatchMode mode) {
    sensorDetected = false;
    trippedBefore = false;
}

void Shamptake::intake(double Power) {
    intakeMotor1.Set(Power);
    //intakeMotor2.Set(Power);
    runIntake = true;
}
void Shamptake::stopIntake() {
    intake(0);
    runIntake = false;
    if (autoIntaking) {
        autoIntaking = false;
    }
}
void Shamptake::autoIntake() {
    autoIntaking = true;
    intakeSpeed = IntakeSpeed::NORMAL;
    runIntakeMotors();
}
void Shamptake::autoShoot() {
    autoShooting = true;
    shooterTimer.Reset();
    shooterTimer.Stop();
    shooter(5000);
}
void Shamptake::shooter(double power) {
    // double motorRightpower = 0;
    // double motorLeftpower = 0;
    // if (shooterMode == Shamptake::ShooterMode::CURVED) { 
    //     motorRightpower = Power * .6;// this should eventually be 6000 rpm
    //     motorLeftpower = Power * .8;// this should eventually be 4000 rpm
    // } else {
    //     motorRightpower = Power * .8;// base speeds
    //     motorLeftpower = Power * .8;
    // }
    //shooterMotorRight.Set(motorRightpower); 
    //shooterMotorLeft.Set(motorLeftpower); 
    targetShooterRPM = power;
    shooterMotorLeftPIDController.SetReference(targetShooterRPM, rev::CANSparkBase::ControlType::kVelocity);
    shooterMotorRightPIDController.SetReference(targetShooterRPM, rev::CANSparkBase::ControlType::kVelocity);
}
bool Shamptake::atTargetRPM() {
    bool isAtTarget = (shooterMotorRightEncoder.GetVelocity() >= targetShooterRPM && shooterMotorLeftEncoder.GetVelocity() >= targetShooterRPM);
    return isAtTarget;
}
void Shamptake::stop() {
    intake(0);
    shooter(0);
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
    sensorDetected = !noteSensor.Get();
    
    if (!sensorDetected) {
        if (trippedBefore) { // Past Sensor
            //sleep(0.7);
            intakeSpeed = IntakeSpeed::STOP;
            if (autoIntaking) {
                autoIntaking = false;
                stopIntake();
            }
        } else { // Before Sensor
            intakeSpeed = IntakeSpeed::NORMAL;
        }
    } else { // Sensor Tripped
        intakeSpeed = IntakeSpeed::SLOW;
        trippedBefore = true;
    }

    if (autoShooting) {
        if (atTargetRPM() && !autoIntaking) {
            intakeSpeed = IntakeSpeed::FIRE;
            shooterTimer.Start();
            autoIntaking = true;
        }
        if (shooterTimer.Get() >= 1_s) {
            autoShooting = false;
            shooterTimer.Stop();
            intakeSpeed = IntakeSpeed::STOP;
            shooter(0);
        }
    }
}

std::string Shamptake::intakeModeString() {
    std::string modeString = "ERROR";
    switch (intakeSpeed)
    {
    case IntakeSpeed::NORMAL:
        modeString = "NORMAL";
        break;
    case IntakeSpeed::STOP:
        modeString = "STOP";
        break;
    case IntakeSpeed::SLOW:
        modeString = "SLOW";
        break;
    case IntakeSpeed::FIRE:
        modeString = "FIRE";
        break;
    case IntakeSpeed::OUTTAKE:
        modeString = "OUTTAKE";
        break;
    default:
        break;
    }
    return modeString;
}

void Shamptake::runIntakeMotors() {
    double speed = 0;
    switch (intakeSpeed)
    {
    case IntakeSpeed::NORMAL:
        speed = 0.7;
        break;
    case IntakeSpeed::STOP:
        speed = 0;
        break;
    case IntakeSpeed::SLOW:
        speed = 0.4;
        break;
    case IntakeSpeed::FIRE:
        speed = 0.8;
        break;
    case IntakeSpeed::OUTTAKE:
        speed = -0.4;
        break;
    default:
        speed = 0;
        break;
    }
    intake(speed);
}