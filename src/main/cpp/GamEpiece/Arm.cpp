#include <GamEpiece/Arm.h>
#include <frc/smartdashboard/SmartDashboard.h>

bool armCanMove = true;
std::string degreesResponse = "not set";
double degrees;

#define ARM_MIN_RAW 0.5
#define ARM_MAX_RAW 0.66

Arm::Arm() 
//: boreEncoder(armMotor.GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, 8192))
//,armEncoder(armMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42))
//: armPIDController(armMotor.GetPIDController())
{
    /*armPIDController.SetP(ARM_MOTOR_P);
    armPIDController.SetI(ARM_MOTOR_I);
    armPIDController.SetD(ARM_MOTOR_D);
    armPIDController.SetIZone(ARM_MOTOR_I_ZONE);
    armPIDController.SetFF(ARM_MOTOR_FEED_FOWARD);
    armPIDController.SetOutputRange(0, 1);*/
    forwardarmLimitSwitch.EnableLimitSwitch(true);
}

Arm::~Arm() {

}

void Arm::process()
{
    double degrees = getBoreDegrees();
    if (degrees <= 180 || degrees >= 216) {
        armCanMove = false;
    } else {
        armCanMove = true;
    }
}

void Arm::sendFeedback() {
    frc::SmartDashboard::PutBoolean("Arm_isOnLowerLimit", isOnLowerLimit());
  //frc::SmartDashboard::PutBoolean("Arm_forwardsparkLimitSwitch", forwardarmLimitSwitch.Get());
  //frc::SmartDashboard::PutBoolean("Arm_reversesparkLimitSwitch", reversearmLimitSwitch.Get());
    frc::SmartDashboard::PutNumber("Arm_borePosition", getRawBorePosition());
    frc::SmartDashboard::PutNumber("Arm_boreDegrees", degrees);
    frc::SmartDashboard::PutBoolean("Arm_canMove", armCanMove);
    frc::SmartDashboard::PutString("Arm_getBoreDegreesResponse", degreesResponse);
    frc::SmartDashboard::PutNumber("Arm_motorTempC", armMotor.GetMotorTemperature());
    frc::SmartDashboard::PutNumber("Arm_motorTempF", armMotor.GetMotorTemperature() * 1.8 + 32);
    frc::SmartDashboard::PutString("Arm_motorMode", getMotorModeString());
    frc::SmartDashboard::PutNumber("Arm_maxspeed", ARM_SLOW_SPEED);    
}

void Arm::doPersistentConfiguration() {
    armMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    armMotor.SetInverted(false);
}

void Arm::resetToMode(MatchMode mode) {

}

bool Arm::isOnLowerLimit() {
    return forwardarmLimitSwitch.Get(); // Get the limit switch reading (it's inverted)
}
bool Arm::init() {
    bool isInit = true;
    return isInit;
}

double Arm::getRawBorePosition() {
    double length = encoder.GetOutput();

    return length;
}

double Arm::getBoreNormalizedPosition() {
    double d = getRawBorePosition();
    d -= ARM_MIN_RAW;
    d /= (ARM_MAX_RAW - ARM_MIN_RAW);

    return d;
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

void Arm::setPower(double power) {
    printf("Incoming Power:%lf\n", power);
    double position = getRawBorePosition();
    if (position > 0.66 && power < 0) {
        power = 0;
        printf("position over %lf limit\n", 0.6);
    } else if (position < 0.5 && power > 0) {
        power = 0;
        printf("position under %lf limit\n", 0.5);
    }
    power = -power;
    printf("3Power:%lf\n", power);

    armMotor.Set(power);
    // if (!backingOffMinimum && getRawMotorRotationPosition() < ARM_MINIMUM_ENCODER) {
    //     power = 0;
    //     backingOffMinimum = true;
    // }
    // printf("1Power:%lf\n", power);
    // if (backingOffMinimum && getRawMotorRotationPosition() >= ARM_MINIMUM_ENCODER_SLOW) {
    //     backingOffMinimum = false;
    // }
    // if (power < 0 && backingOffMinimum) {
    //     power = -ARM_SLOW_SPEED;
    // }


    // if (!backingOffMaximum && getRawMotorRotationPosition() > ARM_MAXIMUM_ENCODER) {
    //     power = 0;
    //     backingOffMaximum = true;
    // } else if (!backingOffMaximum && isOnLowerLimit()) {
    //     power = 0;
    //     backingOffMaximum = true;
    // }
    // printf("2Power:%lf\n", power);
    // if (backingOffMaximum && getRawMotorRotationPosition() <= ARM_MAXIMUM_ENCODER_SLOW) {
    //     backingOffMaximum = false;
    // }
    // if (power > 0 && backingOffMaximum) {
    //     power = ARM_SLOW_SPEED;
    // }

    // if (armCanMove == false) {
    // power = 0;
    // }
}
void Arm::stop() {
    setPower(0);
}