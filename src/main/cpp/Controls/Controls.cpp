#include <Controls/Controls.h>
#include <cmath>
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>

#define AXIS_DEADZONE 0.1
Controls::Controls(Drive* _drive)
:drive(_drive){

}

void Controls::resetToMode(MatchMode mode) { }

void Controls::process() {
    driveController.process();
    auxController.process();


    doSwitchPanel();
    if (callaDisable) {
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
    }
}

void Controls::processInDisabled() {
    doSwitchPanel();

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

bool Controls::getShouldPersistConfig() {
    doSwitchPanel();

    using DriveButton = DriveControllerType::Button;
    using AuxButton = AuxControllerType::Button;

    return settings.isCraterMode
        && driveController.getButton(DriveButton::TRIANGLE) && driveController.getDPad() == ThunderGameController::DPad::DOWN
        && auxController.getButton(AuxButton::CROSS) && auxController.getDPad() == ThunderGameController::DPad::UP;
}

void Controls::doDrive() {
    using DriveButton = DriveControllerType::Button;
    using DriveAxis = DriveControllerType::Axis;

    bool brickDrive = driveController.getButton(DriveButton::CROSS);
    
    bool toggleRotation = driveController.getButton(DriveButton::TRIANGLE, ThunderGameController::ButtonState::PRESSED);
    double xVel = driveController.getLeftXAxis();
    double yVel = driveController.getLeftYAxis();
    double angVel = driveController.getRightXAxis();
    bool xySlowMode = driveController.getLeftBumper();
    bool rotSlowMode = driveController.getRightBumper();
    bool rotSlowerIThinkIDKReallyCallaJustWantedThisForSomeReasonSoHereItIsIGuess = driveController.getRightTrigger() > AXIS_DEADZONE;
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
        // driveCtrlFlags |= Drive::ControlFlag::LOCK_X;
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
        driveAbsAngle = drive->getEstimatedPose().Rotation().Radians();
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

    if (std::fabs(xVel) > AXIS_DEADZONE) {
        finalXVel = improveAxis(xVel);
    }
    if (std::fabs(yVel) > AXIS_DEADZONE) {
        finalYVel = improveAxis(yVel);
    }
    if (std::fabs(angVel) > AXIS_DEADZONE) {
        finalAngVel = improveAxis(angVel);
    }
    if (std::fabs(xAng) > AXIS_DEADZONE) {
        finalXAng = xAng;
    }
    if (std::fabs(yAng) > AXIS_DEADZONE) {
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


    // Hi Peter!!!
    if (xySlowMode || settings.newDriver){
        finalXVel *= 0.375;//.428571439;
        finalYVel *= 0.375;//.427928372; // LOL
    }
    if (rotSlowMode || settings.newDriver){
        finalAngVel *= .5;
    }
    else if (rotSlowerIThinkIDKReallyCallaJustWantedThisForSomeReasonSoHereItIsIGuess || settings.newDriver)  {
        finalAngVel *= .4;
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
    
    if (hangModeControls == true){
        /* hang and trap controls 


        */
       
    } else {

    }
        if (auxController.getButton(AuxButton::TRIANGLE)){
            //  SUBWOOFER preset
        } 
        else if (auxController.getButton(AuxButton::CIRCLE)){
            //  LINE preset
        }
        else if (auxController.getButton(AuxButton::CROSS)){
            //other reserved preset
        }
        else if (auxController.getButton(AuxButton::SQUARE)){
            //amp preset
        } 

        bool overrideGamePieceNo = auxController.getButton(AuxButton::SHARE, ThunderGameController::ButtonState::PRESSED);
        bool overrideGamePieceYes = auxController.getButton(AuxButton::OPTIONS, ThunderGameController::ButtonState::PRESSED);

        if (auxController.getButton(AuxButton::LEFT_BUMPER)){
            if (auxController.getButton(AuxButton::LEFT_TRIGGER_BUTTON)){
                //warm up and then shoot
            } 
            else {
                // just continue warming up
            }
        }
        
        if (auxController.getButton(AuxButton::RIGHT_TRIGGER_BUTTON)){
            //activate intake
        }
        else if (auxController.getButton(AuxButton::RIGHT_BUMPER)){
            //activate outtake
        }




        //CHECK WITH THE MECHIES TO SEE IF THE FOLLOWING FUNT IS ACTUALLY NEEDED
        if (auxController.getDPad() == ThunderGameController::DPad::DOWN){
            //set arm low enough to get under the stage
        } else if (auxController.getDPad() == ThunderGameController::DPad::UP){
            //set arm back to normal position
        }

        double armPivot = -auxController.getAxis(AuxAxis::LEFT_Y);
    


}

void Controls::doAuxManual() {
    using AuxButton = AuxControllerType::Button;
    using AuxAxis = AuxControllerType::Axis;

}

#define SWITCH_LED_DISABLE 1
#define SWITCH_ROBOT_CENTRIC 2
#define SWITCH_HANG_MODE 3
#define SWITCH_BALANCE_CONTROL 4
#define SWITCH_CALLA_DISABLE 5
#define SWITCH_SASHA_DISABLE 6
#define SWITCH_MANUAL_AUX 7
#define SWITCH_CRATER_MODE 8

void Controls::doSwitchPanel() {
    bool ledDisable = switchPanel.GetRawButton(SWITCH_LED_DISABLE);
    driveRobotCentric = switchPanel.GetRawButton(SWITCH_ROBOT_CENTRIC);
    callaDisable = switchPanel.GetRawButton(SWITCH_CALLA_DISABLE);
    sashaDisable = switchPanel.GetRawButton(SWITCH_SASHA_DISABLE);
    manualAux = switchPanel.GetRawButton(SWITCH_MANUAL_AUX);
    settings.isCraterMode = switchPanel.GetRawButton(SWITCH_CRATER_MODE);
    hangModeControls = switchPanel.GetRawButton(SWITCH_HANG_MODE);
    balanceControlOff = switchPanel.GetRawButton(SWITCH_BALANCE_CONTROL);


    int ledMode = frc::SmartDashboard::GetNumber("thunderdashboard_led_mode", 0.0);
}

void Controls::sendFeedback() {

}