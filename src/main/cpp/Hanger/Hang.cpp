#include <Hanger/Hang.h>
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Util/Preferences.h>

Hang::Hang() : 
 hangLeftEncoder(hangMotorLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)) ,
 hangRightEncoder(hangMotorRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42))
 {

}

Hang::~Hang()
{

}

void Hang::enableBrakeMode(bool enabled) {
    if(enabled){
        hangMotorLeft.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
        hangMotorRight.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    }
    else{
        hangMotorLeft.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
        hangMotorRight.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    }
}
void Hang::process() {
    switch (leftMotorState) {
        case BACKTRACKING:
            if (isLeftPawlOpen()) {
                backtrackLeft();
            } else {
                leftMotorState = motorState::IDLE;
            }
            break;
        case AWAITING_CHECK:
            hangMotorLeft.Set(0);
            if (backtrackingLeftCheckTimer.Get() >= 0.3_s) {
                if (isLeftRelayOn()) {
                    if (isLeftPawlOpen()) {
                        backtrackingLeftCheckTimer.Stop();
                        leftMotorState = motorState::BACKTRACKING;
                        setSolenoids(SolenoidStates::OFF);
                        targetLeftEncoderRotation = getLeftMotorPosition() + BACKTRACK_ROTATIONS;
                    } else {
                        backtrackingLeftCheckTimer.Stop();
                        leftMotorState = motorState::IDLE;
                    }
                } else {
                    backtrackingLeftCheckTimer.Stop();
                    leftMotorState = motorState::IDLE;
                }
            }
            break;
        case IDLE:
            hangMotorLeft.Set(0);
            if (isLeftRelayOn() && isLeftPawlOpen()) {
                backtrackingLeftCheckTimer.Restart();
                leftMotorState = motorState::AWAITING_CHECK;
            }
            break;
        case MOVING_UP:
            if (isLeftRelayOn() && !isLeftPawlOpen() && getLeftMotorPosition() >= -PREFERENCE_HANG.MAX_POSTION) {
                hangMotorLeft.Set(-PREFERENCE_CONTROLS.MAX_HANG_UP_SPEED);
            } else {
                hangMotorLeft.Set(0);
            }
            break;
        case MOVING_DOWN:
            if (!isLeftReflectiveSensorTripped()) {
                hangMotorLeft.Set(PREFERENCE_CONTROLS.MAX_HANG_DOWN_SPEED);
            } else {
                hangLeftEncoder.SetPosition(0);
                hangMotorLeft.Set(0);
            }
            break;
    }
    switch (rightMotorState) {
        case BACKTRACKING:
            if (isRightPawlOpen()) {
                backtrackRight();
            } else {
                rightMotorState = motorState::IDLE;
            }
            break;
        case AWAITING_CHECK:
            hangMotorRight.Set(0);
            if (backtrackingRightCheckTimer.Get() >= 0.3_s) {
                if (isRightRelayOn()) {
                    if (isRightPawlOpen()) {
                        backtrackingRightCheckTimer.Stop();
                        rightMotorState = motorState::BACKTRACKING;
                        setSolenoids(SolenoidStates::OFF);
                        targetRightEncoderRotation = getRightMotorPosition() - BACKTRACK_ROTATIONS;
                    } else {
                        backtrackingRightCheckTimer.Stop();
                        rightMotorState = motorState::IDLE;
                    }
                } else {
                    backtrackingRightCheckTimer.Stop();
                    rightMotorState = motorState::IDLE;
                }
            }
            break;
        case IDLE:
            hangMotorRight.Set(0);
            if (isRightRelayOn() && isRightPawlOpen()) {
                backtrackingRightCheckTimer.Restart();
                rightMotorState = motorState::AWAITING_CHECK;
            }
            break;
        case MOVING_UP:
            if (isRightRelayOn() && !isRightPawlOpen() && getRightMotorPosition() <= PREFERENCE_HANG.MAX_POSTION) {
                hangMotorRight.Set(PREFERENCE_CONTROLS.MAX_HANG_UP_SPEED);
            } else {
                hangMotorRight.Set(0);
            }
            break;
        case MOVING_DOWN:
            if (!isRightReflectiveSensorTripped()) {
                hangMotorRight.Set(-PREFERENCE_CONTROLS.MAX_HANG_DOWN_SPEED);
            } else {
                hangMotorRight.Set(0);
                hangRightEncoder.SetPosition(0);
            }
            break;
    }
    // if (backtrackRight) {
    //     if (!isRightPawlOpen()) {
    //         backtrackRight();
    //     }
    // }
}
void Hang::backtrackLeft() {
    if (getLeftMotorPosition() <= targetLeftEncoderRotation) {
        leftMotorState = motorState::AWAITING_CHECK;
        backtrackingLeftCheckTimer.Restart();
        setLeftSolenoid(true);
    } else {
        hangMotorLeft.Set(0.1);
    }
}
void Hang::backtrackRight() {
    if (getRightMotorPosition() >= targetRightEncoderRotation) {
        rightMotorState = motorState::AWAITING_CHECK;
        backtrackingRightCheckTimer.Restart();
        setRightSolenoid(true);
    } else {
        hangMotorRight.Set(-0.1);
    }
}

void Hang::setMotorLeftState(motorState state) { // Motor is reversed
    leftMotorState = state;
}
void Hang::setMotorRightState(motorState state) {
    rightMotorState = state;
}

void Hang::setMotorLeftStateSafe(motorState state) {
    if (leftMotorState != motorState::AWAITING_CHECK && leftMotorState != motorState::BACKTRACKING) {
        setMotorLeftState(state);
    }
}
void Hang::setMotorRightStateSafe(motorState state) {
    if (rightMotorState != motorState::AWAITING_CHECK && rightMotorState != motorState::BACKTRACKING) {
        setMotorRightState(state);
    }
}
void Hang::setRightSolenoid(bool onOff) {
    SolenoidStates currentState = getSolenoidState();
    SolenoidStates nextState = SolenoidStates::OFF;
    if (onOff) {
        if (currentState == SolenoidStates::LEFT || currentState == SolenoidStates::BOTH) {
            nextState = SolenoidStates::BOTH;
        } else {
            nextState = SolenoidStates::RIGHT;
        }
    } else {
        if (currentState == SolenoidStates::LEFT || currentState == SolenoidStates::BOTH) {
            nextState = SolenoidStates::LEFT;
        } else {
            nextState = SolenoidStates::OFF;
        }
    }
}
void Hang::setLeftSolenoid(bool onOff) {
    SolenoidStates currentState = getSolenoidState();
    SolenoidStates nextState = SolenoidStates::OFF;
    if (onOff) {
        if (currentState == SolenoidStates::RIGHT || currentState == SolenoidStates::BOTH) {
            nextState = SolenoidStates::BOTH;
        } else {
            nextState = SolenoidStates::LEFT;
        }
    } else {
        if (currentState == SolenoidStates::RIGHT || currentState == SolenoidStates::BOTH) {
            nextState = SolenoidStates::RIGHT;
        } else {
            nextState = SolenoidStates::OFF;
        }
    }
}
void Hang::setMotorRightSpeed(double speed) {
    if (isRightRelayOn() && isRightPawlOpen()) {
        if (reflectiveHangSensorRight.Get()) {
            hangMotorRight.Set(speed);
        }
        else if (speed > 0) {
            hangMotorRight.Set(speed);
        } 
        else {
            hangMotorRight.Set(0);
        }
    } else {
        hangMotorRight.Set(0);
    }
}
double Hang::getLeftMotorPosition()
{
    hangEncoderLeftPosition = hangLeftEncoder.GetPosition();
    return hangEncoderLeftPosition;
}
double Hang::getRightMotorPosition()
{
    hangEncoderRightPosition = hangRightEncoder.GetPosition();
    return hangEncoderRightPosition;
}
std::string Hang::getMotorLeftModeString() {
    std::string motorMode = "Coast";
    if (hangMotorLeft.GetIdleMode() == rev::CANSparkBase::IdleMode::kBrake) {
        motorMode = "Brake";
    }
    return motorMode;
}
std::string Hang::getMotorRightModeString() {
    std::string motorMode = "Coast";
    if (hangMotorRight.GetIdleMode() == rev::CANSparkBase::IdleMode::kBrake) {
        motorMode = "Brake";
    }
    return motorMode;
}

bool Hang::isLeftReflectiveSensorTripped()
{
    return !reflectiveHangSensorLeft.Get();
}

bool Hang::isRightReflectiveSensorTripped()
{
    return !reflectiveHangSensorRight.Get();
}

bool Hang::isLeftPawlUp()
{
    return !leafSensorLeft.Get();
}

bool Hang::isRightPawlUp()
{
    return !leafSensorRight.Get();
}

void Hang::sendFeedback() {
    frc::SmartDashboard::PutNumber("Hang_Left_Position", -getLeftMotorPosition());
    frc::SmartDashboard::PutNumber("Hang_Right_Position", getRightMotorPosition());
    // frc::SmartDashboard::PutString("Hang_LeftMotorTemp", ConvertTemperatureToString(hangMotorLeft.GetMotorTemperature()));
    // frc::SmartDashboard::PutString("Hang_RightMotorTemp", ConvertTemperatureToString(hangMotorRight.GetMotorTemperature()));
    frc::SmartDashboard::PutString("Hang_LeftmotorMode", getMotorLeftModeString());
    frc::SmartDashboard::PutString("Hang_LeftMotorState", getMotorStateString(leftMotorState));
    frc::SmartDashboard::PutString("Hang_RightmotorMode", getMotorRightModeString());
    frc::SmartDashboard::PutString("Hang_RightMotorState", getMotorStateString(rightMotorState));
    frc::SmartDashboard::PutBoolean("Hang_LeftRRSensor", reflectiveHangSensorLeft.Get());
    frc::SmartDashboard::PutNumber("Hang_Left_Timer", backtrackingLeftCheckTimer.Get().value());
    frc::SmartDashboard::PutBoolean("Hang_isLeftPawlOpen", isLeftPawlOpen());
    frc::SmartDashboard::PutBoolean("Hang_RightRRSensor", reflectiveHangSensorRight.Get());
    frc::SmartDashboard::PutNumber("Hang_Right_Timer", backtrackingRightCheckTimer.Get().value());
    frc::SmartDashboard::PutBoolean("Hang_isRightPawlOpen", isRightPawlOpen());
    frc::SmartDashboard::PutString("Hang_SolenoidStates", getSolenoidStateString());
    // frc::SmartDashboard::PutString("Leaf_Sensor_Right")
}
void Hang::setSolenoids(Hang::SolenoidStates state) {
    solenoidRelay.Set((frc::Relay::Value)state);
}
std::string Hang::getMotorStateString(motorState state) {
    std::string backtrackStateString = "IDLE";
    switch (state)
    {
    case motorState::AWAITING_CHECK:
        backtrackStateString = "Awaiting Check";
        break;
    case motorState::BACKTRACKING:
        backtrackStateString = "Backtracking";
        break;
    case motorState::MOVING_UP:
        backtrackStateString = "Moving Up";
        break;
    case motorState::MOVING_DOWN:
        backtrackStateString = "Moving Down";
        break;
    default:
        break;
    }
    return backtrackStateString;
}
Hang::SolenoidStates Hang::getSolenoidState() {
    return (SolenoidStates)solenoidRelay.Get();
}
std::string Hang::getSolenoidStateString() {
    std::string solenoidState = "Off";
    switch (getSolenoidState())
    {
    case SolenoidStates::BOTH:
        solenoidState = "Both";
        break;
    case SolenoidStates::LEFT:
        solenoidState = "Left Only";
        break;
    case SolenoidStates::RIGHT:
        solenoidState = "Right Only";
        break;
    default:
        break;
    }
    return solenoidState;
}

bool Hang::isLeftRelayOn() {
    SolenoidStates state = (SolenoidStates)solenoidRelay.Get(); 
    return (state == SolenoidStates::LEFT || state == SolenoidStates::BOTH);
}
bool Hang::isRightRelayOn() {
    SolenoidStates state = (SolenoidStates)solenoidRelay.Get();
    return (state == SolenoidStates::RIGHT || state == SolenoidStates::BOTH);
}
bool Hang::isLeftPawlOpen() {
    return leafSensorLeft.Get();
}
bool Hang::isRightPawlOpen() {
    return leafSensorRight.Get();
}
std::string Hang::ConvertTemperatureToString(double temp_c) {
    double temp_f = temp_c * 1.8 + 32;
    std::string temperature = std::to_string(temp_c) + "C " + std::to_string(temp_f) + "F";
    return temperature;
}