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
 hangLeftEncoder(hangMotorLeft.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)) ,
 hangRightEncoder(hangMotorRight.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42))
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

void Hang::process () {}

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

// void Hang::debug(Feedback* feedback) {

//     feedback->sendDouble("thunderdashboard", "hang_pos", 100 * (winch->GetEncoder() / kMaxHeight));

//     feedback->sendDouble("hang", "speed", winch->Get());
//     feedback->sendDouble("hang", "encoder", winch->GetEncoder());
//     feedback->sendDouble("hang", "ratchet", ratchetPawlMarried);
//     feedback->sendString("hang", "zero sensor", leafSensor.Get() ? "true" : "false");

//     const char* stateName = "";
//     switch(currentState) {
//         case STOPPED:
//             stateName = "STOPPED";
//         break;
//         case DIVORCED:
//             stateName = "WINCH_DISENGAGE";
//         break;
//         case MOVING_UP:
//             stateName = "MOVING_UP";
//         break;
//         case MOVING_DOWN:
//             stateName = "MOVING_DOWN";
//         break;
//     }
//     feedback->sendString("hang", "state", stateName);

//     const char* directionName = "";
//     switch(moveDirection) {
//         case UP:
//             directionName = "UP";
//         break;
//         case STOP:
//             directionName = "STOP";
//         break;
//         case DOWN:
//             directionName = "DOWN";
//         break;
//     }
//     feedback->sendString("hang", "direction", directionName);
//     feedback->sendBoolean("hang", "sensor broken", zeroSensorBroke);
//     feedback->sendBoolean("hang", "reflective sensor tripped", hangSensorTripped);
// }
// /*
// */
// void Hang::zeroSensorBroken(bool broke){
//     zeroSensorBroke = broke;
// }

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
void Hang::setMotorLeftSpeed(double speed){
    hangMotorLeft.Set(speed);
}
void Hang::setMotorRightSpeed(double speed){
    hangMotorRight.Set(speed);
}
void Hang::setSpeed(double speed){
    setMotorLeftSpeed(speed);
    setMotorRightSpeed(speed);
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

void Hang::sendFeedback() {
    // frc::SmartDashboard::PutBoolean("Arm_isOnLowerLimit", isOnLowerLimit());
    // frc::SmartDashboard::PutNumber("Arm_borePosition", getRawBorePosition());
    frc::SmartDashboard::PutNumber("Hang_Left_Position", getLeftMotorPosition());
    frc::SmartDashboard::PutNumber("Hang_Right_Position", getRightMotorPosition());
    frc::SmartDashboard::PutString("Hang_LeftMotorTemp", ConvertTemperatureToString(hangMotorLeft.GetMotorTemperature()));
    frc::SmartDashboard::PutString("Hang_RightMotorTemp", ConvertTemperatureToString(hangMotorRight.GetMotorTemperature()));
    frc::SmartDashboard::PutString("Hang_LeftmotorMode", getMotorLeftModeString());
    frc::SmartDashboard::PutString("Hang_RightmotorMode", getMotorRightModeString());
}
std::string Hang::ConvertTemperatureToString(double temp){
    char temperature[16];
    sprintf(temperature, "%lf C/%lf F", temp, temp * 1.8 + 32);
    return temperature;
}