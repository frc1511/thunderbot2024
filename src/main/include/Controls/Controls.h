#pragma once

#include <Drive/Drive.h>
#include <Basic/IOMap.h>
#include <Controls/PS4Controller.h>
#include <Controls/XboxController.h>
#include <Hanger/Hang.h>
#include <GamEpiece/Arm.h>
#include <GamEpiece/Shamptake.h>

using DriveControllerType = ThunderPS4Controller;
using AuxControllerType = ThunderPS4Controller;

class Controls : public Mechanism {
public:
    double MAX_ARM_SPEED = 0.5;
    double MAX_HANG_SPEED = 0.2;
    //Controls(Drive* drive, Shamptake* _shamptake, Arm* _arm, Hang* _hang);
 Controls(Drive* drive, Shamptake* _shamptake, Arm* _arm, Hang* _hang);

    void resetToMode(MatchMode mode) override;
    void process() override;
    void sendFeedback() override;

    void processInDisabled();
    bool getShouldPersistConfig();
    
private:
    Drive* drive;
    DriveControllerType driveController{ThunderGameController::Controller::DRIVER};
    Shamptake* shamptake;
    Arm* arm;
    Hang* hang;
    AuxControllerType auxController{ThunderGameController::Controller::AUX};
    frc::GenericHID switchPanel{2};
    bool armMode;
    double currentSpeed;
    double armSpeed;
    double hangMotorLeft;
    double hangMotorRight;
    bool dpadUp;
    bool dpadRight;
    bool dpadDown;
    bool dpadLeft;
    bool leftSolenoidManualButton;
    bool rightSolenoidManualButton;

    void doDrive();
    void doAux();
    void doAuxManual();
    void doSwitchPanel();
    bool driveLockX = false;
    bool manualAux = false;
    bool doUltraBrickMode = false;
    bool shouldStrobe = false;
    bool hangModeControls = false;
    bool driveRobotCentric = true;
    bool balanceControlOff = false;
    unsigned driveCtrlFlags = 0;

    bool driveAbsRotation = false;
    units::radian_t driveAbsAngle = 0_deg;

    // CHANGE NAMES!
    bool callaDisable = false;
    bool sashaDisable = false;
};