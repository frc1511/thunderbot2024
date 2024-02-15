#include <GamEpiece/Arm.h>
#include <frc/smartdashboard/SmartDashboard.h>
Arm::Arm(): armEncoder(armMotor.GetEncoder(rev::RelativeEncoder::EncoderType::kHallSensor, 42)) {
    
}

Arm::~Arm() {

}

void Arm::process()
{

}

void Arm::sendFeedback() {
    frc::SmartDashboard::PutBoolean("Arm_isOnLowerLimit", isOnLowerLimit());
    frc::SmartDashboard::PutNumber("Arm_rawMotorPosition", getRawMotorPosition());
    frc::SmartDashboard::PutNumber("Arm_angleMotorPosition", getRawMotorRotationPosition());
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

}
double Arm::getRawMotorPosition() {
    double position = -armEncoder.GetPosition(); // Encoders are reversed
    return position;
}

double Arm::getRawMotorRotationPosition() {
    double rotation = -armEncoder.GetPosition();
    return rotation;
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