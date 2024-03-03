#pragma once
#include <frc/DigitalInput.h>
#include <frc/Relay.h>
#include <frc/Timer.h>
#include <Basic/IOMap.h>
#include <rev/CANSparkMax.h>
#include <Basic/Mechanism.h>

// #include <ThunderSparkMax/ThunderSparkMax.h>
// #include <Feedback/Feedback.h>
// #include <BlinkyBlinky/BlinkyBlinky.h>

#define BACKTRACK_ROTATIONS -2.072

class Hang : public Mechanism {
    public:
    Hang();
    ~Hang();


    enum HangMovement {
        UP, 
        STOP, 
        DOWN
    };

    void reset();
    void process();
//     void debug(Feedback* feedback);
//     //void lights(Lights* lights);

    enum motorState {
        IDLE,
        BACKTRACKING,
        AWAITING_CHECK,
        MOVING_UP,
        MOVING_DOWN
    };

    /**
     *  Moves the mechanism to the target position 
     */
    void move(HangMovement direction);

    void setMotorLeftState(motorState state);
    void setMotorLeftStateSafe(motorState state);

    void setMotorRightState(motorState state);
    void setMotorRightStateSafe(motorState state);

    void setRightSolenoid(bool onOff);
    void setLeftSolenoid(bool onOff);

    void setMotorRightSpeed(double speed);

    void enableBrakeMode(bool enabled);

    double getLeftMotorPosition();
    double getRightMotorPosition();

    std::string getMotorLeftModeString();
    std::string getMotorRightModeString();

    std::string getSolenoidStateString();

    std::string ConvertTemperatureToString(double temp);

    enum SolenoidStates {
        LEFT = frc::Relay::Value::kReverse,
        RIGHT = frc::Relay::Value::kForward,
        BOTH = frc::Relay::Value::kOn,
        OFF = frc::Relay::Value::kOff,
    };

    void setSolenoids(Hang::SolenoidStates state);
    void sendFeedback() override;

    bool isLeftReflectiveSensorTripped();
    bool isRightReflectiveSensorTripped();
    bool isLeftPawlUp();
    bool isRightPawlUp();

    bool isLeftRelayOn();
    bool isRightRelayOn();

    bool isLeftPawlOpen();
    bool isRightPawlOpen();

//     /**
//      * When retracting, move slower for precision
//      */
//     void enableSlowRetract(bool slowModeEnabled);

//     /**
//      * broke is true if the zero sensor is broken
//      * broke is false if the zero sensor isnt broken
//      */
//     void zeroSensorBroken(bool broke); // NOT NEEDED

//     void reflectiveHangSensorTripped(bool reflectiveHangSensorisTripped);

//     void setHangIdleMode(bool idleModeEnabled);

    motorState leftMotorState = motorState::IDLE;
    motorState rightMotorState = motorState::IDLE;

private:
    rev::CANSparkMax hangMotorLeft {CAN_HANG_ARM_LEFT, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder hangLeftEncoder;
    rev::CANSparkMax hangMotorRight {CAN_HANG_ARM_RIGHT, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder hangRightEncoder;

    frc::Relay solenoidRelay {RELAY_HANG_GEAR_LOCK, frc::Relay::kBothDirections};

    frc::DigitalInput reflectiveHangSensorLeft {DIO_HANG_RR_SENSOR_LEFT};
    frc::DigitalInput reflectiveHangSensorRight {DIO_HANG_RR_SENSOR_RIGHT};

    frc::DigitalInput leafSensorLeft {DIO_HANG_LIMIT_SWITCH_LEFT};
    frc::DigitalInput leafSensorRight {DIO_HANG_LIMIT_SWITCH_RIGHT};

    double hangEncoderLeftPosition;
    double hangEncoderRightPosition;

    double targetRightEncoderRotation = 0;
    double targetLeftEncoderRotation = 0;

    frc::Timer backtrackingLeftCheckTimer;
    frc::Timer backtrackingRightCheckTimer;

    void backtrackLeft();
    void backtrackRight();

    std::string getMotorStateString(motorState state);
    SolenoidStates getSolenoidState();
//     bool lastSensorReading = false; 

//     HangMovement moveDirection = STOP; // Default movement is stop
//     bool slowRetract = false; 

//     enum HangState {
//         STOPPED,
//         DIVORCED,
//         MOVING_UP,
//         MOVING_DOWN
//     };

//     HangState currentState = STOPPED; // Default state is stopped

//     double moveDownRamp = 0; // ???

//     void setRatchetPawlMarried(bool married);
//     bool ratchetPawlMarried = false; // Default is not engaged

//     double preEngageEncoder = 0;

//     bool zeroSensorBroke = false;

//     bool hangSensorTripped = false;
//
};