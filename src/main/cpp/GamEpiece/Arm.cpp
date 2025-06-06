#include <GamEpiece/Arm.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Util/Preferences.h>
#include <Basic/Settings.h>

Arm::Arm() {
    configureMotors();
}

Arm::~Arm() {
    
}
void Arm::configureMotors() {
    armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    armMotor.SetInverted(false);
    armBrake.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    armBrake.SetInverted(false);
    encoder.SetDistancePerRotation(360);
    armPIDController.Reset(getBoreDegrees());
    forwardarmLimitSwitch.EnableLimitSwitch(true);
    reversearmLimitSwitch.EnableLimitSwitch(true);
}
void Arm::process()
{
    if (!settings.isCraterMode) {
        units::degree_t degrees = getBoreDegrees();
        if (isNearPreset(Presets::AMP) && targetAngle == presetAngles[Presets::AMP]) {
            armPIDController.SetP(PREFERENCE_ARM.AMPEND_PID.Kp);
            armPIDController.SetI(PREFERENCE_ARM.AMPEND_PID.Ki);
            armPIDController.SetD(PREFERENCE_ARM.AMPEND_PID.Kd);
        }
        if (degrees >= -1_deg && degrees <= 101_deg) {
            double power = armPIDController.Calculate(degrees, targetAngle);
            power += cosf((degrees / 180_deg) * M_PI) * 0.08;
            setPower(-power);
        } else {
            setPower(0);
        }
    }
    doBrake();
}

void Arm::sendFeedback() {
    frc::SmartDashboard::PutNumber("Arm_rawBorePosition", getRawBorePosition().value());
    frc::SmartDashboard::PutNumber("Arm_boreDegrees", double(getBoreDegrees()));
    frc::SmartDashboard::PutNumber("Arm_motorTempC", armMotor.GetMotorTemperature());
    frc::SmartDashboard::PutString("Arm_motorMode", getMotorModeString());  
    frc::SmartDashboard::PutNumber("Arm_targetAngle", targetAngle.value());
    frc::SmartDashboard::PutBoolean("Arm_atTargetAngle", isMoveDone());
    frc::SmartDashboard::PutBoolean("Arm_legal", withinLegalLimit());
    frc::SmartDashboard::PutBoolean("Arm_Braked", braked);
    frc::SmartDashboard::PutBoolean("Arm_NearAMP", isNearPreset(Presets::AMP));
    frc::SmartDashboard::PutData("Arm_PID_Controller", &armPIDController);
}
bool Arm::withinLegalLimit() {
    bool legal = false;
    units::degree_t degrees = getBoreDegrees();
    if (degrees <= PREFERENCE_ARM.MAX_LEGAL_LIMIT && degrees >= PREFERENCE_ARM.MIN_LEGAL_LIMIT) {
        legal = true;
    }
    return legal;
}
void Arm::doPersistentConfiguration() {
    armMotor.RestoreFactoryDefaults();
    armBrake.RestoreFactoryDefaults();
    configureMotors();
    armMotor.BurnFlash();
    armBrake.BurnFlash();
}

void Arm::resetToMode(MatchMode mode) {
    setMotorBrake(true);
    stop();
    armPIDController.Reset(getBoreDegrees());
    if (mode == MatchMode::AUTO || mode == MatchMode::TELEOP) {
        //disengageBrake();
        moveToAngle(getBoreDegrees() + 10_deg);
    }
}

void Arm::setMotorBrake(bool armBrakeOn) {
    rev::CANSparkBase::IdleMode currentMode = armMotor.GetIdleMode();
    if (armBrakeOn) {
        if (currentMode != rev::CANSparkBase::IdleMode::kBrake) {
            armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
        }
    } else {
        if (currentMode != rev::CANSparkBase::IdleMode::kCoast) {
            armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
        }
    }
}

units::degree_t Arm::getRawBorePosition() {
    return 360_deg - units::degree_t(encoder.GetDistance());
}

double Arm::getBoreNormalizedPosition() {
    double d = getBoreDegrees().value() / 85.0;
    return d;
}

units::degree_t Arm::getBoreDegrees() {
    units::degree_t degrees = getRawBorePosition();
    return units::math::fmod(degrees - PREFERENCE_ARM.ENCODER_OFFSET, 360_deg);
}

bool Arm::isAtLowerLimit() {
    return forwardarmLimitSwitch.Get();
}

bool Arm::isAtUpperLimit() {
    return reversearmLimitSwitch.Get();
}
std::string Arm::getMotorModeString() {
    std::string motorMode = "Coast";
    if (armMotor.GetIdleMode() == rev::CANSparkBase::IdleMode::kBrake) {
        motorMode = "Brake";
    }
    return motorMode;
}

void Arm::moveToAngle(units::angle::degree_t angle) {
    targetAngle = std::clamp(angle, 0_deg, 87_deg);
}

void Arm::moveToPreset(Presets preset) {
    units::angle::degree_t movingTo = presetAngles[preset];
    if (preset == Presets::AMP) { // Normal PID will not work when at the AMP position, use this to configure PID for AMP
        armPIDController.SetP(PREFERENCE_ARM.AMP_PID.Kp);
        armPIDController.SetI(PREFERENCE_ARM.AMP_PID.Ki);
        armPIDController.SetD(PREFERENCE_ARM.AMP_PID.Kd);
    } else {
        armPIDController.SetP(PREFERENCE_ARM.PID.Kp);
        armPIDController.SetI(PREFERENCE_ARM.PID.Ki);
        armPIDController.SetD(PREFERENCE_ARM.PID.Kd);
    }
    moveToAngle(movingTo);
}

bool Arm::isMoveDone() {
    //if the arm is at or past the point it needs to be at, then it is done going to the position
    if (fabs(double(getBoreDegrees()) - targetAngle.value()) <= targetAngleThreshold) {
        return true;
        //don't stop moving the arm since it will just fall back down
    }
    return false;
}

bool Arm::isNearPreset(Presets preset) {
    return fabs((getBoreDegrees() - presetAngles[preset]).value()) <= presetAngleThreshold;
}

void Arm::setPower(double power) {
    armMotor.Set(power);
}

void Arm::engageBrake() {
    currentBrakeMode = BrakeModes::ENGAGE;
}

void Arm::disengageBrake() {
    currentBrakeMode = BrakeModes::DISENGAGE;
}

void Arm::manualBrakePower(double power) {
    armBrake.Set(power);
}

void Arm::doBrake() {
    if (currentBrakeMode == BrakeModes::DISENGAGE) {
        if (braked) {
        brakeTimer.Reset();
        armBrake.Set(-.05);
        brakeTimer.Start();
        if (brakeTimer.Get() >= 1_s) {
            armBrake.Set(0.0);
            braked = false;
            brakeTimer.Stop();
            currentBrakeMode = BrakeModes::NONE;
        }
    }
    } else if (currentBrakeMode == BrakeModes::ENGAGE) {
        if (!braked) {
        brakeTimer.Reset();
        armBrake.Set(.05);
        brakeTimer.Start();
        if (brakeTimer.Get() >= 1_s) {
            armBrake.Set(0.0);
            braked = true;
            brakeTimer.Stop();
            currentBrakeMode = BrakeModes::NONE;
        }
    }
    }
}

void Arm::stop() {
    setPower(0);
}