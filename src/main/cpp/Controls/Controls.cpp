#include <Controls/Controls.h>
#include <cmath>
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <BlinkyBlinky/BlinkyBlinky.h>


Controls::Controls(Drive* _drive, Shamptake* _shamptake, Arm* _arm, Hang* _hang, BlinkyBlinky* _blink, Limelight* _limelight, bool* _debugMode):
    drive(_drive),
    shamptake(_shamptake),
    arm(_arm),
    hang(_hang),
    blink(_blink),
    limelight(_limelight),
    armMode(true),
    debugMode(_debugMode)
{

}

void Controls::resetToMode(MatchMode mode) { }

void Controls::process() {
    driveController.process();
    auxController.process();

    doSwitchPanel(false);
    if (callaDisable) {
        printf("===DRIVE DISABLED===\n");
        drive->manualControlRelRotation(0, 0, 0, Drive::ControlFlag::BRICK);
    }
    else {
        doDrive();
    }

    if (!sashaDisable) {
        if (manualAux) {
            doAuxManual();
        }
        else {
            doAux();
        }
    } else {
        printf("===AUX DISABLED===\n");
        shamptake->controlProcess(false, false, false, false, false, false, false);
    }
}

void Controls::processInDisabled() {
    doSwitchPanel(true);

    using DriveButton = DriveControllerType::Button;

    bool resetOdometry = driveController.getButton(DriveButton::OPTIONS, ThunderGameController::ButtonState::PRESSED);
    bool calIMU = driveController.getButton(DriveButton::SHARE, ThunderGameController::ButtonState::PRESSED);

    if (resetOdometry) {
        drive->resetOdometry();
    }

    if (calIMU) {
        drive->calibrateIMU();
    }
}

bool Controls::getShouldPersistConfig() { //this will flash ALL motor controllers, use sparingly and probably only when swapping controllers
    doSwitchPanel(false);

    using DriveButton = DriveControllerType::Button;
    using AuxButton = AuxControllerType::Button;

    return settings.isCraterMode && driveController.getButton(DriveButton::CROSS, ThunderGameController::ButtonState::PRESSED) && auxController.getButton(AuxButton::CROSS, ThunderGameController::ButtonState::PRESSED);
}

void Controls::doDrive() {
    using DriveButton = DriveControllerType::Button;
    using DriveAxis = DriveControllerType::Axis;

    bool brickDrive = driveController.getButton(DriveButton::CROSS);
    
    bool toggleRotation = driveController.getButton(DriveButton::TRIANGLE, ThunderGameController::ButtonState::PRESSED);
    ampLight = driveController.getButton(DriveButton::SQUARE);
    sourceLight = driveController.getButton(DriveButton::TOUCH_PAD);
    double xVel = driveController.getLeftXAxis();
    double yVel = driveController.getLeftYAxis();
    double angVel = driveController.getRightXAxis();
    bool xySlowMode = driveController.getLeftBumper();
    bool rotSlowMode = driveController.getRightBumper();
    bool rotSlowerIThinkIDKReallyCallaJustWantedThisForSomeReasonSoHereItIsIGuess = false;//driveController.getRightTrigger() > PREFERENCE_CONTROLS.AXIS_DEADZONE;
    shouldStrobe = driveController.getButton(DriveButton::CIRCLE);

    double xAng = driveController.getAxis(DriveAxis::RIGHT_X);
    double yAng = driveController.getAxis(DriveAxis::RIGHT_Y);

    bool resetOdometry = driveController.getButton(DriveButton::OPTIONS, ThunderGameController::ButtonState::PRESSED);
    bool calIMU = driveController.getButton(DriveButton::SHARE, ThunderGameController::ButtonState::PRESSED);


    if (driveController.getButton(DriveButton::LEFT_STICK, ThunderGameController::ButtonState::PRESSED)) {
        driveLockX = !driveLockX;
    }

    bool wasBrickDrive = driveCtrlFlags & Drive::ControlFlag::BRICK;
    
    driveCtrlFlags = Drive::ControlFlag::NONE;

    if (!driveRobotCentric) {
        driveCtrlFlags |= Drive::ControlFlag::FIELD_CENTRIC;
    }

    if (brickDrive) {
        driveCtrlFlags |= Drive::ControlFlag::BRICK;
    }

    if (driveLockX) {
        //driveCtrlFlags |= Drive::ControlFlag::LOCK_Y;
    }

    

    if (toggleRotation) {
        drive->resetPIDControllers();
        if (!driveAbsRotation) {
            driveAbsAngle = drive->getEstimatedPose().Rotation().Radians();
            driveAbsRotation = true;
        }
        else {
            driveAbsRotation = false;
        }
    }

    if (resetOdometry) {
        drive->resetOdometry();
        //driveAbsAngle = drive->getEstimatedPose().Rotation().Radians();
    }

    if (calIMU) {
        drive->calibrateIMU();
        driveAbsAngle = drive->getEstimatedPose().Rotation().Radians();
    }


    double finalXVel = 0.0,
           finalYVel = 0.0,
           finalAngVel = 0.0,
           finalXAng = 0.0,
           finalYAng = 0.0;

    // Improves the joystick axis to be smoother and easier to control.
    auto improveAxis = [](double axis) -> double {
        return std::sin(axis * (std::numbers::pi / 2.0));
    };

    if (std::fabs(xVel) > PREFERENCE_CONTROLS.AXIS_DEADZONE) {
        finalXVel = improveAxis(xVel);
    }
    if (std::fabs(yVel) > PREFERENCE_CONTROLS.AXIS_DEADZONE) {
        finalYVel = improveAxis(yVel);
    }
    if (std::fabs(angVel) > PREFERENCE_CONTROLS.AXIS_DEADZONE) {
        finalAngVel = improveAxis(angVel);
    }
    if (std::fabs(xAng) > PREFERENCE_CONTROLS.AXIS_DEADZONE) {
        finalXAng = xAng;
    }
    if (std::fabs(yAng) > PREFERENCE_CONTROLS.AXIS_DEADZONE) {
        finalYAng = yAng;
    }

    // Returns whether the robot should be moving.
    auto isMoving = [&]() -> bool {
        bool r = driveAbsRotation ? finalXAng || finalYAng : finalAngVel;
        return finalXVel || finalYVel || r;
    };

    // Stay in brick drive mode if the robot isn't moving.
    if (wasBrickDrive && !isMoving()) {
        driveCtrlFlags |= Drive::ControlFlag::BRICK;
    }


    // Hi Peter!!
    if (xySlowMode || settings.newDriver){
        finalXVel *= 0.375;//.428571439;
        finalYVel *= 0.375;//.427928372; // LOL
    }
    if (rotSlowMode || settings.newDriver){
        finalAngVel *= .36;
    }
    else if (rotSlowerIThinkIDKReallyCallaJustWantedThisForSomeReasonSoHereItIsIGuess || settings.newDriver)  {
        finalAngVel *= .26;
    }

    // Control the drivetrain.
    if (driveAbsRotation) {
        // If changed rotation.
        if (finalYAng || finalXAng) {
            // Calculate the new absolute rotation for the robot.
            driveAbsAngle = units::radian_t(std::atan2(-finalYAng, finalXAng)) - 90_deg;
        }

        drive->manualControlAbsRotation(finalXVel, -finalYVel, driveAbsAngle, driveCtrlFlags);
    }
    else {
        drive->manualControlRelRotation(finalXVel, -finalYVel, -finalAngVel, driveCtrlFlags);
    }
}
void Controls::doAux() {
    using AuxButton = AuxControllerType::Button;
    using AuxAxis = AuxControllerType::Axis;
    
    //CONTROLMAP
    const bool dpadUp = auxController.getDPad() == ThunderGameController::DPad::UP;
    const bool dpadRight = auxController.getDPad() == ThunderGameController::DPad::RIGHT;
    const bool dpadDown = auxController.getDPad() == ThunderGameController::DPad::DOWN;
    const bool dpadLeft = auxController.getDPad() == ThunderGameController::DPad::LEFT;


    bool overrideGamePieceNo = auxController.getButton(AuxButton::SHARE, ThunderGameController::ButtonState::PRESSED);
    bool overrideGamePieceYes = auxController.getButton(AuxButton::OPTIONS, ThunderGameController::ButtonState::PRESSED);

    bool intake = auxController.getAxis(AuxAxis::RIGHT_TRIGGER) > 0.5;//Intake, for running intake motors
    bool shooter = auxController.getButton(AuxButton::LEFT_BUMPER);//Preheat, for running shooter motors
    bool fire = auxController.getAxis(AuxAxis::LEFT_TRIGGER) > 0.5;//Shoot, for running shooter and intake motors
    bool outtake = auxController.getButton(AuxButton::RIGHT_BUMPER);//Outtake, for running intake motors in reverse
    
    //double armSpeed = -auxController.getAxis(AuxAxis::LEFT_Y);//Arm movement, gets speed for arm movement + direction
    bool stagePreset = auxController.getButton(AuxButton::CROSS);
    bool linePreset = auxController.getButton(AuxButton::CIRCLE);
    bool subwooferPreset = auxController.getButton(AuxButton::TRIANGLE);
    bool ampPreset = auxController.getButton(AuxButton::SQUARE);
    bool intakePreset = dpadDown;
    bool travelPreset = auxController.getButton(AuxButton::TOUCH_PAD);
    if (intakePreset) {
        arm->moveToPreset(Arm::BASE);
    } else if (stagePreset) {
        arm->moveToPreset(Arm::STAGE);
    } else if (linePreset) {
        arm->moveToPreset(Arm::LINE);
    } else if (subwooferPreset) {
        arm->moveToPreset(Arm::SUBWOOFER);
    } else if (ampPreset) {
        arm->moveToPreset(Arm::AMP);
    } else if (travelPreset) {
        arm->moveToPreset(Arm::TRAVEL);
    }


    bool rightHang = dpadRight;
    bool leftHang = dpadLeft;
    bool bothHang = dpadUp;

    double hangMotorLeft = -auxController.getAxis(AuxAxis::LEFT_Y);
    double hangMotorRight = -auxController.getAxis(AuxAxis::RIGHT_Y);

    if (std::fabs(hangMotorLeft) < PREFERENCE_CONTROLS.AXIS_DEADZONE) {
        hangMotorLeft = 0;
    }

    if (std::fabs(hangMotorRight) < PREFERENCE_CONTROLS.AXIS_DEADZONE) {
        hangMotorRight = 0;
    }

    if (hangMotorLeft > 0) {
        leftHang = true;
    }

    if (hangMotorRight > 0) {
        rightHang = true;
    }

    if (hang != nullptr)
    {
        if ((rightHang && leftHang) || bothHang) {//TEMP, to be removed after hang gets automatic solenoids when doing motors (manual mode right now)
            hang->setSolenoids(Hang::SolenoidStates::BOTH);
        } else if (leftHang) {
            hang->setSolenoids(Hang::SolenoidStates::LEFT);
        } else if (rightHang) {
            hang->setSolenoids(Hang::SolenoidStates::RIGHT);
        } else {
            hang->setSolenoids(Hang::SolenoidStates::OFF);
        }

        if (hangMotorLeft > 0) {
            hang->setMotorLeftStateSafe(Hang::motorState::MOVING_UP);
        } else if (hangMotorLeft < 0) {
            hang->setMotorLeftStateSafe(Hang::motorState::MOVING_DOWN);
        } else {
            hang->setMotorLeftStateSafe(Hang::motorState::IDLE);
        }
        

        if (hangMotorRight > 0) {
            hang->setMotorRightStateSafe(Hang::motorState::MOVING_UP);
        } else if (hangMotorRight < 0) {
            hang->setMotorRightStateSafe(Hang::motorState::MOVING_DOWN);
        } else {
            hang->setMotorRightStateSafe(Hang::motorState::IDLE);
        }
    }

    //SHAMPTAKE
    bool halfCourt = driveController.getRightTrigger() > PREFERENCE_CONTROLS.AXIS_DEADZONE;
    shamptake->controlProcess(intake, outtake, fire, shooter, overrideGamePieceYes, overrideGamePieceNo, halfCourt);
}

void Controls::doAuxManual() {
    using AuxButton = AuxControllerType::Button;
    using AuxAxis = AuxControllerType::Axis;

}

#define SWITCH_LED_ENABLE 1
#define SWITCH_ROBOT_CENTRIC 2
#define SWITCH_LIMELIGHT_ENABLE 3
#define SWITCH_BALANCE_CONTROL 4
#define SWITCH_CALLA_DISABLE 5
#define SWITCH_SASHA_DISABLE 6
#define SWITCH_MANUAL_AUX 7
#define SWITCH_CRATER_MODE 8
#define SWITCH_ARM_BRAKE 9
#define SWITCH_UNUSED 10
#define SWITCH_DEBUG_MODE 11

void Controls::doSwitchPanel(bool isDissabled) {
    bool ledEnable = switchPanel.GetRawButton(SWITCH_LED_ENABLE);
    driveRobotCentric = switchPanel.GetRawButton(SWITCH_ROBOT_CENTRIC);
    callaDisable = switchPanel.GetRawButton(SWITCH_CALLA_DISABLE);
    sashaDisable = switchPanel.GetRawButton(SWITCH_SASHA_DISABLE);
    manualAux = switchPanel.GetRawButton(SWITCH_MANUAL_AUX);
    settings.isCraterMode = switchPanel.GetRawButton(SWITCH_CRATER_MODE);
    balanceControlOff = switchPanel.GetRawButton(SWITCH_BALANCE_CONTROL);
    armBrakeDissable = switchPanel.GetRawButton(SWITCH_ARM_BRAKE);
    *debugMode = switchPanel.GetRawButton(SWITCH_DEBUG_MODE);

    limelight->limelightEnabled = switchPanel.GetRawButton(SWITCH_LIMELIGHT_ENABLE);

    if (armBrakeDissable) {
        if (isDissabled) {
            arm->setMotorBrake(false);
        } else {
            arm->setMotorBrake(true);
        }
    } else {
        arm->setMotorBrake(true);
    }
    int ledMode = frc::SmartDashboard::GetNumber("thunderdashboard_led_mode", 0.0);
    
    if (ledEnable){
        blink->setLEDMode(BlinkyBlinky::LEDMode::OFF);
    }
    else if (settings.isCraterMode){
        blink->setLEDMode(BlinkyBlinky::LEDMode::PIT_MODE);
    }
    else if (hangModeControls){
        blink->setLEDMode(BlinkyBlinky::LEDMode::HANG_MODE);
    }
    else if (shouldStrobe){
        blink->setLEDMode(BlinkyBlinky::LEDMode::PARTY);
    }
    else if (fire){
        blink->setLEDMode(BlinkyBlinky::LEDMode::SCORE);
    }
    else if (shamptake->hasGamepiece()){
        blink->setLEDMode(BlinkyBlinky::LEDMode::HAS_GAMEPIECE);
    }
    else if (ampLight){
        blink->setLEDMode(BlinkyBlinky::LEDMode::AMP);
    }
    else if (!drive->isIMUCalibrated()){
        blink->setLEDMode(BlinkyBlinky::LEDMode::CALIBRATING);
    }
    else if (sourceLight){
        blink->setLEDMode(BlinkyBlinky::LEDMode::SOURCE);
    }
    else if (shamptake->intakeSpeed == Shamptake::IntakeSpeed::NORMAL_INTAKE){
        blink->setLEDMode(BlinkyBlinky::LEDMode::INTAKE);
    }
    else {
        if (getCurrentMode () == MatchMode::DISABLED){
            blink->setLEDMode(BlinkyBlinky::LEDMode::ALLIANCE);
        }
        else {
            blink->setLEDMode(BlinkyBlinky::LEDMode::BASE);
        }

    }
}






void Controls::sendFeedback() {
    frc::SmartDashboard::PutString("Arm_currentmode", armMode ? "arm mode" : "hang mode");
    //frc::SmartDashboard::PutNumber("Arm_Speed", armSpeed);

}