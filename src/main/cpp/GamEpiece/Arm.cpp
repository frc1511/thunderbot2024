#include <GamEpiece/Arm.h>
#include <frc/smartdashboard/SmartDashboard.h>
Arm::Arm(): armEncoder(armMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)),
            boreEncoder(armMotor.GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, 8192)) {
    
}

Arm::~Arm() {

}

void Arm::process()
{

}

void Arm::sendFeedback() {
    frc::SmartDashboard::PutBoolean("Arm_isOnLowerLimit", isOnLowerLimit());
    frc::SmartDashboard::PutNumber("Arm_borePosition", getRawBorePosition());
    frc::SmartDashboard::PutNumber("Arm_angleMotorPosition", getRawMotorRotationPosition());
    frc::SmartDashboard::PutNumber("Arm_rawMotorPosition", getRawMotorPosition());
    frc::SmartDashboard::PutNumber("Arm_motorTempC", armMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Arm_motorTempF", armMotor.GetMotorTemperature() * 1.8 + 32);
    frc::SmartDashboard::PutString("Arm_motorMode", getMotorModeString());
}

void Arm::doPersistentConfiguration() {
    armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    armMotor.SetInverted(true);
}

void Arm::resetToMode(MatchMode mode) {

}

bool Arm::isOnLowerLimit() {
    return !limitSwitch.Get(); // Get the limit switch reading (it's inverted)
}
bool Arm::init() {
    bool isInit = true;
    return isInit;
}
double Arm::getRawMotorPosition() {
    double position = -armEncoder.GetPosition(); // Encoders are reversed
    return position;
}

double Arm::getRawMotorRotationPosition() {
    double rotation = -armEncoder.GetPosition();
    return rotation;
}

double Arm::getRawBorePosition() {
    double rotations = boreEncoder.GetPosition();
    return rotations;
}

std::string Arm::getMotorModeString() {
    std::string motorMode = "Coast";
    if (armMotor.GetIdleMode() == rev::CANSparkBase::IdleMode::kBrake) {
        motorMode = "Brake";
    }
    return motorMode;
}

void Arm::setPower(double power) {
    if (!backingOffMinimum && getRawMotorRotationPosition() < ARM_MINIMUM_ENCODER) {
        power = 0;
        backingOffMinimum = true;
    }
    if (backingOffMinimum && getRawMotorRotationPosition() >= ARM_MINIMUM_ENCODER_SLOW) {
        backingOffMinimum = false;
    }
    if (power < 0 && backingOffMinimum) {
        power = -ARM_SLOW_SPEED;
    }


    if (!backingOffMaximum && getRawMotorRotationPosition() > ARM_MAXIMUM_ENCODER) {
        power = 0;
        backingOffMaximum = true;
    } else if (!backingOffMaximum && isOnLowerLimit()) {
        power = 0;
        backingOffMaximum = true;
    }
    if (backingOffMaximum && getRawMotorRotationPosition() <= ARM_MAXIMUM_ENCODER_SLOW) {
        backingOffMaximum = false;
    }
    if (power > 0 && backingOffMaximum) {
        power = ARM_SLOW_SPEED;
    }
    armMotor.Set(power);
}
void Arm::stop() {
    setPower(0);
}