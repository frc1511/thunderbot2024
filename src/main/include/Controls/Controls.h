#pragma once

#include <Drive/Drive.h>
#include <Controls/PS4Controller.h>
#include <Controls/XboxController.h>

using DriveControllerType = ThunderPS4Controller;
using AuxControllerType = ThunderPS4Controller;

class Controls : public Mechanism {
public:
    Controls(Drive* drive);

    void resetToMode(MatchMode mode) override;
    void process() override;
    void sendFeedback() override;

    void processInDisabled();
    bool getShouldPersistConfig();
    
private:
    Drive* drive;
    DriveControllerType driveController{ThunderGameController::Controller::DRIVER};
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

    bool driveRobotCentric = false;
    unsigned driveCtrlFlags = 0;

    bool driveAbsRotation = false;
    units::radian_t driveAbsAngle = 0_deg;

    // CHANGE NAMES!
    bool callaDisable = false;
    bool auxDisable = false;
};