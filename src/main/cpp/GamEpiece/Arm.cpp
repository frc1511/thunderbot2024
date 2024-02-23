#include <GamEpiece/Arm.h>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>


Arm::Arm() {

}

Arm::~Arm() {

}

void Arm::process()
{
    double degrees = getBoreDegrees();
    if (degrees <= 180 || degrees >= 235) {
        armCanMove = false;
    } else {
        armCanMove = true;
    }

    double power = armPIDController.Calculate(units::degree_t(degrees), targetAngle);
    setPower(power);
}

void Arm::sendFeedback() {
  //frc::SmartDashboard::PutBoolean("Arm_forwardsparkLimitSwitch", forwardarmLimitSwitch.Get());
  //frc::SmartDashboard::PutBoolean("Arm_reversesparkLimitSwitch", reversearmLimitSwitch.Get());
    frc::SmartDashboard::PutNumber("Arm_borePosition", getRawBorePosition());
    frc::SmartDashboard::PutNumber("Arm_boreDegrees", getBoreDegrees());
    frc::SmartDashboard::PutBoolean("Arm_canMove", armCanMove);
    frc::SmartDashboard::PutNumber("Arm_motorTempC", armMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Arm_motorTempF", armMotor.GetMotorTemperature() * 1.8 + 32);
    frc::SmartDashboard::PutString("Arm_motorMode", getMotorModeString());  
    frc::SmartDashboard::PutNumber("Arm_targetAngle", targetAngle.value());
}

void Arm::doPersistentConfiguration() {
    armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    armMotor.SetInverted(false);
}

void Arm::resetToMode(MatchMode mode) {

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

double Arm::getRawBorePosition() {
    double length = encoder.GetOutput();

    return length;
}

double Arm::getBoreDegrees() {
    double degrees = getRawBorePosition();
    degrees *= 360.0;
    return degrees;
}

std::string Arm::getMotorModeString() {
    std::string motorMode = "Coast";
    if (armMotor.GetIdleMode() == rev::CANSparkBase::IdleMode::kBrake) {
        motorMode = "Brake";
    }
    return motorMode;
}

void Arm::moveToAngle(units::angle::degree_t angle) {
    targetAngle = angle;
    printf("Angle target set to: %f\n", targetAngle.value());
}

bool Arm::isMoveDone() {
    //if the arm is at or past the point it needs to be at, then it is done going to the position
    if (fabs(getBoreDegrees()) >= fabs(double(targetAngle))) {
        return true;
        //don't stop moving the arm since it will just fall back down
    }
    return false;
}

void Arm::setPower(double power) {
    power = -power;
    printf("Incoming Power:%lf\n", power);
    double position = getRawBorePosition();
    if (position > 0.66 && power < 0) {
        power = 0;
        printf("position over %lf limit\n", 0.6);
    } else if (position < 0.5 && power > 0) {
        power = 0;
        printf("position under %lf limit\n", 0.5);
    }
    printf("Setting Power:%lf\n", power);

    armMotor.Set(power);
}
void Arm::stop() {
    setPower(0);
}