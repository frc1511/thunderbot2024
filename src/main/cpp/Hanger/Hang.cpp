#include <Hanger/Hang.h>
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>

// #include <Feedback/Feedback.h>

// // Height Thresholds, using neo shaft rotations
// //UP
// const double kMaxHeight = 560; // will not go up above this height 
// const double kFastUpHeight = 460; // will drive up faster for range that will not be stopped in

// //DOWN
// const double kMaxRetractHeight = 0; // will not go down below this height
// const double kParkHeight = 50; // goes extremely slowly below this height
// const double kFastDownHeight = 100; // when moving down, it will get faster until this value, at which point the robot should be off the ground
// const double kPrecisionDownHeight = 460; // when moving down, it will initially move slowly until this limit is reached

// const double kExtendSpeed = .3; // fine precision up speed
// const double kRetractSpeed = .3; // fine precision down speed

// const double kDisengageThreshold = 2; // 22 degrees / 360 degrees = .06 rotations (10)
// const double kDisengageSpeed = .2; // absolute

// const double kFastUpSpeed = 1; // fast up speed .8
// const double kNormalUpSpeed = .5; // normal up speeed .5

// const double kPrecisionDownSpeed = .2;
// const double kFastDownSpeed = 1; // fast down speed until off the ground 1

// const double kSolenoidEngaged = .3; // .75
// const double kSolenoidDisengaged = .72; // .2

 Hang::Hang() : 
 hangLeftEncoder(hangMotorLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)) ,
 hangRightEncoder(hangMotorRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42))
 {
//     setRatchetPawlMarried(true);
//     // winch->SetEncoder(0);
//     // winch->SetInverted(true);
//     // winch->SetIdleMode(ThunderSparkMax::BRAKE);
//     // winch->SetOpenLoopRampRate(.5);
}

Hang::~Hang()
{

}


// void Hang::process() {
//     switch(currentState) { // if the current state any of these, then call do whichever process is in the body
//         case STOPPED: // if the current state is stopped
//             winch->Set(0); // stop the motor from moving 
//             setRatchetPawlMarried(true); // Engage the ratchet and pawl 

//             if(moveDirection == UP && winch->GetEncoder() < kMaxHeight) { // if moving up and the distance that the motor moved is shorter than the max height, 
//                 currentState = DIVORCED; // Disengage the ratchet and pawl
//                 preEngageEncoder = winch->GetEncoder(); // Get the encoder value
//             }
            
//             if(moveDirection == DOWN && winch->GetEncoder() > kMaxRetractHeight) { // if moving down and the distance the motor moves is greater than the min height
//                 currentState = MOVING_DOWN;
//             }
//             break;
//         case DIVORCED: // If not engaged
//             setRatchetPawlMarried(false); // set the ratchet and pawl to be disengaged
//             if(winch->GetEncoder() > preEngageEncoder-kDisengageThreshold) { // Disengage threshold???
//                 winch->Set(-fabs(kDisengageSpeed));
//             }
//             else if(winch->GetEncoder() <= preEngageEncoder-kDisengageThreshold) {
//                 winch->Set(0);
//                 currentState = MOVING_UP;
//             }
//             break;
//         case MOVING_UP:
//             setRatchetPawlMarried(false);
//             if(winch->GetEncoder() < kFastUpHeight) { // if the motor moves less than the fastMax height
//                 winch->Set(kFastUpSpeed); // make the motor move faster
//             }
//             else if(winch->GetEncoder() < kMaxHeight) { // if it is greater, but less than the Max height
//                 winch->Set(kNormalUpSpeed); // move normal speed
//             }
//             else { // if the encoder is greater than the max height
//                 winch->Set(0);  // stop it from moving
//                 currentState = STOPPED;
//             }

//             if(moveDirection != UP) { // if it isn't moving up but the movement direction reads up
//                 currentState = STOPPED; // then stop it from moving
//             }
//             break;
//         case MOVING_DOWN: // mostly the same as up but reverse
//             if(winch->GetEncoder() > kFastDownHeight) {
//                 if(!slowRetract) {
//                     winch->Set(-kFastDownSpeed);
//                 }
//                 else {
//                     winch->Set(-kPrecisionDownSpeed);
//                 }
//             }
//             else if(winch->GetEncoder() > kMaxRetractHeight) {
//                 if(winch->GetEncoder() < kParkHeight) {
//                     // winch->Set(-kParkDownSpeed);
//                 }
//                 else {
//                     winch->Set(-kRetractSpeed);
//                 }
//             }        
            
//             if(winch->GetEncoder() < kMaxRetractHeight) {
//                 currentState = STOPPED;
//             }

//             if(moveDirection != DOWN) {
//                 currentState = STOPPED;
//             }            

//         break;
//     }

//     if(!zeroSensorBroke) {
//         if(lastSensorReading == true && leafSensor.Get() == false) {
//             winch->SetEncoder(0);
//         }
//         lastSensorReading = leafSensor.Get(); // look into
//     }
// }

// void Hang::reset() {

// }

// void Hang::move(Hang::HangMovement position) {
//     moveDirection = position;
// } // look into

// void Hang::reflectiveHangSensorTripped(bool reflectiveHangSensorisTripped){
//     hangSensorTripped = reflectiveHangSensorisTripped;
// }

// void Hang::enableSlowRetract(bool enable) {
//     slowRetract = enable;
// }

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
            if (!isLeftReflectiveSensorTripped()) {
                hangMotorLeft.Set(-0.2);
            }
            break;
        case MOVING_DOWN:
            if (isLeftRelayOn() && !isLeftPawlOpen() && !isLeftReflectiveSensorTripped()) {
                hangMotorLeft.Set(0.2);
            } else {
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
            if (!isLeftReflectiveSensorTripped()) {
                hangMotorLeft.Set(0.2);
            }
            break;
        case MOVING_DOWN:
            if (isRightRelayOn() && !isRightPawlOpen() && !isRightReflectiveSensorTripped()) {
                hangMotorRight.Set(-0.2);
            } else {
                hangMotorRight.Set(0);
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
    frc::SmartDashboard::PutNumber("Hang_Left_Position", getLeftMotorPosition());
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

    // Dashboard hang page (Implement once range of motion is determined).
    frc::SmartDashboard::PutNumber("thunderdashboard_2024_hang_left_percent", 0.0); // getLeftMotorPosition() / MAX_HANG_POSITION
    frc::SmartDashboard::PutNumber("thunderdashboard_2024_hang_right_percent", 0.0); // getRightMotorPosition() / MAX_HANG_POSITION
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