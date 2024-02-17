#pragma once
#include <frc/DigitalInput.h>
#include <Basic/IOMap.h>
#include <rev/CANSparkMax.h>
#include <ThunderSparkMax/ThunderSparkMax.h>
#include <Feedback/Feedback.h>
#include <BlinkyBlinky/BlinkyBlinky.h>

class Hang {
    public:
    Hang();

    enum HangMovement {
        UP, 
        STOP, 
        DOWN
    };

    void reset();
    void process();
    void debug(Feedback* feedback);
    //void lights(Lights* lights);

    /**
     *  Moves the mechanism to the target position 
     */
    void move(HangMovement direction);

    /**
     * When retracting, move slower for precision
     */
    void enableSlowRetract(bool slowModeEnabled);

    /**
     * broke is true if the zero sensor is broken
     * broke is false if the zero sensor isnt broken
     */
    void zeroSensorBroken(bool broke); // NOT NEEDED

    void reflectiveHangSensorTripped(bool reflectiveHangSensorisTripped);

    void setHangIdleMode(bool idleModeEnabled);

    private:
    ThunderSparkMax *winch;    
    //frc::Servo ratchet{PWM_HANG_RATCHET}; ---NOT NEEDED

    frc::DigitalInput reflectiveHangSensor{DIO_HANG_RR_SENSOR_LEFT};

    frc::DigitalInput leafSensor{DIO_HANG_LIMIT_SWITCH_LEFT};
    bool lastSensorReading = false; 

    HangMovement moveDirection = STOP; // Default movement is stop
    bool slowRetract = false; 

    enum HangState {
        STOPPED,
        DIVORCED,
        MOVING_UP,
        MOVING_DOWN
    };

    HangState currentState = STOPPED; // Default state is stopped

    double moveDownRamp = 0; // ???

    void setRatchetPawlMarried(bool married);
    bool ratchetPawlMarried = false; // Default is not engaged

    double preEngageEncoder = 0;

    bool zeroSensorBroke = false;

    bool hangSensorTripped = false;
};