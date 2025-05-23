#pragma once

#include <Drive/Drive.h>
#include <Basic/IOMap.h>
#include <Controls/PS4Controller.h>
#include <Controls/XboxController.h>
#include <Hanger/Hang.h>
#include <GamEpiece/Arm.h>
#include <GamEpiece/Shamptake.h>
#include <Controls/Controls.h>
#include <BlinkyBlinky/BlinkyBlinky.h>
#include <Util/Limelight.h>
#include <Util/Preferences.h>

using DriveControllerType = ThunderPS4Controller;
using AuxControllerType = ThunderPS4Controller;

class Controls : public Mechanism {
public:
    //Controls(Drive* drive, Shamptake* _shamptake, Arm* _arm, Hang* _hang);
    Controls(Drive* drive, Shamptake* _shamptake, Arm* _arm, Hang* _hang, BlinkyBlinky* _blink, Limelight* _limelight, bool* _debugMode);

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
    BlinkyBlinky* blink;
    Limelight* limelight;
    bool* debugMode;
    AuxControllerType auxController{ThunderGameController::Controller::AUX};
    frc::GenericHID switchPanel{2};
    bool armMode;

    void doDrive();
    void doAux();
    void doAuxManual();
    void doSwitchPanel(bool isDissabled);
    bool ledEnable = false;
    bool driveLockX = false;
    bool manualAux = false;
    bool ampLight = false;
    bool doUltraBrickMode = false;
    bool shouldStrobe = false;
    bool hangModeControls = false;
    bool driveRobotCentric = false;
    bool balanceControlOff = false;
    bool armBrakeDissable = false;
    bool sourceLight = false;
    bool fire = false;
    unsigned driveCtrlFlags = 0;

    bool driveAbsRotation = false;
    units::radian_t driveAbsAngle = 0_deg;

    // CHANGE NAMES!
    bool callaDisable = false;
    bool sashaDisable = false;
};