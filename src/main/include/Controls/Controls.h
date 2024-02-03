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
    Controls(Drive* drive, Shamptake* _shamptake);

    void resetToMode(MatchMode mode) override;
    void process() override;
    void sendFeedback() override;

    void processInDisabled();
    bool getShouldPersistConfig();
    
private:
    Drive* drive;
    DriveControllerType driveController{ThunderGameController::Controller::DRIVER};
    Shamptake* shamptake;
    AuxControllerType auxController{ThunderGameController::Controller::AUX};
    frc::GenericHID switchPanel{2};
    
    void doDrive();
    void doAux();
    void doAuxManual();
    void doSwitchPanel();
    bool driveLockX = false;
    bool manualAux = false;
    bool doUltraBrickMode = false;
    bool shouldStrobe = false;
    bool hangModeControls = false;
    bool driveRobotCentric = false;
    bool balanceControlOff = false;
    unsigned driveCtrlFlags = 0;

    bool driveAbsRotation = false;
    units::radian_t driveAbsAngle = 0_deg;

    // CHANGE NAMES!
    bool callaDisable = false;
    bool sashaDisable = false;
};