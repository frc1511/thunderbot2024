#include <GamEpiece/Arm.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Util/Preferences.h>
#include <basic/Settings.h>

Arm::Arm() {
    armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    armMotor.SetInverted(false);
    encoder.SetDistancePerRotation(360);
    armPIDController.Reset(getBoreDegrees());
    forwardarmLimitSwitch.EnableLimitSwitch(true);
}

Arm::~Arm() {
    
}

void Arm::process()
{
    if (!settings.isCraterMode) {
        units::degree_t degrees = getBoreDegrees();

        double power = armPIDController.Calculate(degrees, targetAngle);
        setPower(-power);
    }
}

void Arm::sendFeedback() {
    frc::SmartDashboard::PutNumber("Arm_rawBorePosition", getRawBorePosition().value());
    frc::SmartDashboard::PutNumber("Arm_boreDegrees", double(getBoreDegrees()));
    //frc::SmartDashboard::PutNumber("Arm_motorTempC", armMotor.GetMotorTemperature());
    //frc::SmartDashboard::PutNumber("Arm_motorTempF", armMotor.GetMotorTemperature() * 1.8 + 32);
    frc::SmartDashboard::PutString("Arm_motorMode", getMotorModeString());  
    frc::SmartDashboard::PutNumber("Arm_targetAngle", targetAngle.value());
    frc::SmartDashboard::PutBoolean("Arm_NearAMP", isNearPreset(Presets::AMP));
}

void Arm::doPersistentConfiguration() {
    
}

void Arm::resetToMode(MatchMode mode) {
    setMotorBrake(true);
    stop();
    armPIDController.Reset(getBoreDegrees());
}

void Arm::setMotorBrake(bool armBrakeOn) {
    if (armBrakeOn) {
        armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    } else {
        armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
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
    return fabs(double(targetAngle - presetAngles[preset])) <= presetAngleThreshold;
}

void Arm::setPower(double power) {
    armMotor.Set(power);
}
void Arm::stop() {
    setPower(0);
}