#include <GamEpiece/Arm.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define ARM_ENCODER_OFFSET 116.28_deg

Arm::Arm() {
    armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    armMotor.SetInverted(false);
    encoder.SetDistancePerRotation(360);
    armPIDController.Reset(getBoreDegrees());
}

Arm::~Arm() {
    
}

void Arm::process()
{
    units::degree_t degrees = getBoreDegrees();

    double power = armPIDController.Calculate(degrees, targetAngle);
    setPower(-power);
}

void Arm::sendFeedback() {
    frc::SmartDashboard::PutNumber("Arm_rawBorePosition", getRawBorePosition().value());
    frc::SmartDashboard::PutNumber("Arm_boreDegrees", double(getBoreDegrees()));
    frc::SmartDashboard::PutNumber("Arm_motorTempC", armMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Arm_motorTempF", armMotor.GetMotorTemperature() * 1.8 + 32);
    frc::SmartDashboard::PutString("Arm_motorMode", getMotorModeString());  
    frc::SmartDashboard::PutNumber("Arm_targetAngle", targetAngle.value());
}

void Arm::doPersistentConfiguration() {
    
}

void Arm::resetToMode(MatchMode mode) {
    setMotorBrake(true);
    setPower(0);
    armPIDController.Reset(getBoreDegrees());
}

bool Arm::init() {
    bool isInit = true;
    return isInit;
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

units::degree_t Arm::getBoreDegrees() {
    units::degree_t degrees = getRawBorePosition();
    return units::math::fmod(degrees - ARM_ENCODER_OFFSET, 360_deg);
}

std::string Arm::getMotorModeString() {
    std::string motorMode = "Coast";
    if (armMotor.GetIdleMode() == rev::CANSparkBase::IdleMode::kBrake) {
        motorMode = "Brake";
    }
    return motorMode;
}

void Arm::moveToAngle(units::angle::degree_t angle) {
    targetAngle = std::clamp(angle, 0_deg, 85_deg);
}

bool Arm::isMoveDone() {
    //if the arm is at or past the point it needs to be at, then it is done going to the position
    if (fabs(double(getBoreDegrees())) >= fabs(double(targetAngle))) {
        return true;
        //don't stop moving the arm since it will just fall back down
    }
    return false;
}

bool Arm::isAtAmp() {
    return targetAngle >= 70_deg;
}

void Arm::setPower(double power) {
    armMotor.Set(power);
}
void Arm::stop() {
    setPower(0);
}