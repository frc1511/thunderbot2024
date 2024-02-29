#include <Controls/Controls.h>
#include <cmath>
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <BlinkyBlinky/BlinkyBlinky.h>

#define AXIS_DEADZONE 0.1


Controls::Controls(Drive* _drive, Shamptake* _shamptake, Arm* _arm, Hang* _hang, BlinkyBlinky* _blink, bool* _debugMode):
    drive(_drive),
    shamptake(_shamptake),
    arm(_arm),
    hang(_hang),
    blink(_blink),
    armMode(true),
    debugMode(_debugMode)
{

}

void Controls::resetToMode(MatchMode mode) { }

void Controls::process() {
    driveController.process();
    auxController.process();
    doAux();


    doSwitchPanel(false);
    if (callaDisable) {
        drive->manualControlRelRotation(0, 0, 0, Drive::ControlFlag::BRICK);
    }
    else {
        doDrive();
    }

    // if (!sashaDisable) {
    //     if (manualAux) {
    //         doAuxManual();
    //     }
    //     else {
    //         doAux();
    //     }
    // }
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

bool Controls::getShouldPersistConfig() {
    doSwitchPanel(false);

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
    ampLight = driveController.getButton(DriveButton::SQUARE);
    sourceLight = driveController.getButton(DriveButton::TOUCH_PAD);
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


    if (armMode) {
        //double armSpeed = -auxController.getAxis(AuxAxis::LEFT_Y);//Arm movement, gets speed for arm movement + direction
        bool intakePreset = auxController.getButton(AuxButton::CROSS);
        bool linePreset = auxController.getButton(AuxButton::CIRCLE);
        bool subwooferPreset = auxController.getButton(AuxButton::TRIANGLE);
        bool ampPreset = auxController.getButton(AuxButton::SQUARE);

        if (intakePreset) {
            arm->moveToPreset(Arm::MEDIUM);
        } else if (linePreset) {
            arm->moveToPreset(Arm::LINE);
        } else if (subwooferPreset) {
            arm->moveToPreset(Arm::SUBWOOFER);
        } else if (ampPreset) {
            arm->moveToPreset(Arm::AMP); // Don't forget to update Arm::isAtAmp()
        }
    } else { //Hang mode
        double hangMotorLeft = -auxController.getAxis(AuxAxis::LEFT_Y);//Hang movement
        double hangMotorRight = -auxController.getAxis(AuxAxis::RIGHT_Y);

        //stuff for when hang gets automatic solenoids when doing motors
        //bool armSubwooferMode = auxController.getButton(AuxButton::Y);
        //bool armLineMode
    }

    //SHAMPTAKE
    if (!intake) {
        shamptake->intakeSpeed = shamptake->STOP_INTAKE;
    }

    if (shooter) {
        if (fire) {
            shamptake->intakeSpeed = shamptake->FIRE_INTAKE;
            shamptake->hasNote = false;
            shamptake->trippedBefore = false;
        }
        if (arm->isNearPreset(Arm::Presets::AMP)) {
            shamptake->shooterSpeed = shamptake->AMP_SHOOTER;
        } else {
            shamptake->shooterSpeed = shamptake->FIRE_SHOOTER;
        }
    } else {
        shamptake->stopShooter();
    }
    
    if (outtake) {
        shamptake->intakeSpeed = shamptake->OUTTAKE_INTAKE;
        shamptake->hasNote = false;
        shamptake->trippedBefore = false;
    }


   /* if (!armmode){
        //HANG
        // Hang stuff - MAKE A FUNCTION OUTTA THIS STUFF
        

        if (std::fabs(hangMotorLeft) < AXIS_DEADZONE) {
            hangMotorLeft = 0;
        }

        if (hangMotorLeft > MAX_HANG_SPEED) {
            hangMotorLeft = MAX_HANG_SPEED;
        }
        if (hangMotorLeft < -MAX_HANG_SPEED) {
            hangMotorLeft = -MAX_HANG_SPEED;
        }

        if (std::fabs(hangMotorRight) < AXIS_DEADZONE) {
            hangMotorRight = 0;
        }

        if (hangMotorRight > MAX_HANG_SPEED) {
            hangMotorRight = MAX_HANG_SPEED;
        }
        if (hangMotorRight < -MAX_HANG_SPEED) {
            hangMotorRight = -MAX_HANG_SPEED;
        }
        if (hang != nullptr)
        {
            if (otherPreset && linePreset) {//TEMP, to be removed after hang gets automatic solenoids when doing motors (manual mode right now)
                hang->setSolenoids(Hang::SolenoidStates::BOTH);
            } else if (otherPreset) {
                hang->setSolenoids(Hang::SolenoidStates::LEFT);
            } else if (linePreset) {
                hang->setSolenoids(Hang::SolenoidStates::RIGHT);
            } else {
                hang->setSolenoids(Hang::SolenoidStates::OFF);
            }
            hang->setMotorLeftSpeed(-hangMotorLeft);
            hang->setMotorRightSpeed(hangMotorRight);
        }
    }
    // if (armModeToggle){
    //     armMode = !armMode;
    //     armModeToggle = false;
    // }
    //HANG
    if (hangModeControls == true){
         hang and trap controls 


        
       
    }*/
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
#define SWITCH_ARM_BRAKE 9
#define SWITCH_UNUSED 10
#define SWITCH_DEBUG_MODE 11

void Controls::doSwitchPanel(bool isDissabled) {
    bool ledDisable = switchPanel.GetRawButton(SWITCH_LED_DISABLE);
    driveRobotCentric = switchPanel.GetRawButton(SWITCH_ROBOT_CENTRIC);
    callaDisable = switchPanel.GetRawButton(SWITCH_CALLA_DISABLE);
    sashaDisable = switchPanel.GetRawButton(SWITCH_SASHA_DISABLE);
    manualAux = switchPanel.GetRawButton(SWITCH_MANUAL_AUX);
    settings.isCraterMode = switchPanel.GetRawButton(SWITCH_CRATER_MODE);
    hangModeControls = switchPanel.GetRawButton(SWITCH_HANG_MODE);
    balanceControlOff = switchPanel.GetRawButton(SWITCH_BALANCE_CONTROL);
    armBrakeDissable = switchPanel.GetRawButton(SWITCH_ARM_BRAKE);
    *debugMode = switchPanel.GetRawButton(SWITCH_DEBUG_MODE);

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

    if (ledDisable){
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
    else if (shamptake->intakeSpeed == Shamptake::IntakeSpeed::NORMAL_INTAKE){
        blink->setLEDMode(BlinkyBlinky::LEDMode::INTAKE);
    }
    else if (shamptake->hasNote){
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
    frc::SmartDashboard::PutNumber("Hang_Speed", MAX_ARM_SPEED);
    //frc::SmartDashboard::PutNumber("Arm_Speed", armSpeed);

}