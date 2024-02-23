#include <Controls/Controls.h>
#include <cmath>
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>

#define AXIS_DEADZONE 0.1


Controls::Controls(Drive* _drive, Shamptake* _shamptake, Arm* _arm, Hang* _hang) :
    drive(_drive),
    shamptake(_shamptake),
    arm(_arm),
    hang(_hang),
    armMode(true) 
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


    // Hi Peter!!
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
    
    //CONTROLMAP
    dpadUp = false;
    dpadRight = false;
    dpadDown = false;
    dpadLeft = false;
    if (auxController.getDPad() == ThunderGameController::DPad::UP) {
        dpadUp = true;
    } else if (auxController.getDPad() == ThunderGameController::DPad::RIGHT) {
        dpadRight = true;
    } else if (auxController.getDPad() == ThunderGameController::DPad::DOWN) {
        dpadDown = true;
    } else if (auxController.getDPad() == ThunderGameController::DPad::LEFT) {
        dpadLeft = true;
    }


    bool overrideGamePieceNo = auxController.getButton(AuxButton::SHARE, ThunderGameController::ButtonState::PRESSED);
    bool overrideGamePieceYes = auxController.getButton(AuxButton::OPTIONS, ThunderGameController::ButtonState::PRESSED);

    bool intake = auxController.getAxis(AuxAxis::RIGHT_TRIGGER) > 0.5;//Intake, for running intake motors
    bool shooter = auxController.getButton(AuxButton::LEFT_BUMPER);//Preheat, for running shooter motors
    bool fire = auxController.getAxis(AuxAxis::LEFT_TRIGGER) > 0.5;//Shoot, for running shooter and intake motors
    bool outtake = auxController.getButton(AuxButton::RIGHT_BUMPER);//Outtake, for running intake motors in reverse


    bool armNormal = false;
    if (armMode) {
        double armSpeed = -auxController.getAxis(AuxAxis::LEFT_Y);//Arm movement, gets speed for arm movement + direction
        bool otherPreset = auxController.getButton(AuxButton::CROSS);
        bool linePreset = auxController.getButton(AuxButton::CIRCLE);
        bool subwooferPreset = auxController.getButton(AuxButton::TRIANGLE);
        bool ampPreset = auxController.getButton(AuxButton::SQUARE);
        bool armNormal = dpadUp;
    } else if (!armMode) {
        double hangMotorLeft = -auxController.getAxis(AuxAxis::LEFT_Y);//Hang movement
        double hangMotorRight = -auxController.getAxis(AuxAxis::RIGHT_Y);
        
        //TEMP
        bool otherPreset = auxController.getButton(AuxButton::CROSS);//TEMP these are eventually going to be arm settings, change after hang gets automatic solenoids when doing motors
        bool linePreset = auxController.getButton(AuxButton::CIRCLE);
        bool subwooferPreset = auxController.getButton(AuxButton::TRIANGLE);
        bool ampPreset = auxController.getButton(AuxButton::SQUARE);


        //stuff for when hang gets automatic solenoids when doing motors
        //bool armSubwooferMode = auxController.getButton(AuxButton::Y);
        //bool armLineMode
    }

    //SHAMPTAKE
    if (!intake) {
        shamptake->intakeSpeed = shamptake->STOP;
    }

    if (shooter) {
        if (fire){
            shamptake->intakeSpeed = shamptake->FIRE;
            shamptake->shooter(1);
            shamptake->trippedBefore = false;
            printf("RESET\n");
        } else {
            shamptake->shooter(0.6);
        }
    } else {
        shamptake->shooter(0);
    }
    
    if (outtake) {
        shamptake->intakeSpeed = shamptake->OUTTAKE;
        shamptake->trippedBefore = false;
        printf("RESET\n");
    }


    //ARM/HANG
    if (armMode){
        // Arm stuff- ALSO  A FUNCTIONNNN OUTTA DIS STUFF 2

        if (arm != nullptr)
        {
            if (otherPreset){
                //set arm low enough to get under the stage
            } 
            else if (subwooferPreset){//These aren't going to be set until after hang solenoids stuff is ready (solenoids open when they need to without input)
                //  SUBWOOFER preset
            } 
            else if (linePreset){
                //  LINE preset
            }
            else if (ampPreset){
                arm->moveToAngle(200_deg);
                //amp preset
            }
            else if (armNormal){
                //set arm back to normal position
                arm->moveToAngle(230_deg);
            }
        }

    }   
    else{
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
        /* hang and trap controls 


        */
       
    }
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

void Controls::doSwitchPanel(bool isDissabled) {
    bool ledDisable = switchPanel.GetRawButton(SWITCH_LED_DISABLE);
    driveRobotCentric = switchPanel.GetRawButton(SWITCH_ROBOT_CENTRIC);
    callaDisable = switchPanel.GetRawButton(SWITCH_CALLA_DISABLE);
    sashaDisable = switchPanel.GetRawButton(SWITCH_SASHA_DISABLE);
    manualAux = switchPanel.GetRawButton(SWITCH_MANUAL_AUX);
    settings.isCraterMode = switchPanel.GetRawButton(SWITCH_CRATER_MODE);
    hangModeControls = switchPanel.GetRawButton(SWITCH_HANG_MODE);
    balanceControlOff = switchPanel.GetRawButton(SWITCH_BALANCE_CONTROL);

    if (balanceControlOff) {
        if (isDissabled) {
            arm->setMotorBrake(false);
        } else {
            arm->setMotorBrake(true);
        }
    } else {
        arm->setMotorBrake(true);
    }

    int ledMode = frc::SmartDashboard::GetNumber("thunderdashboard_led_mode", 0.0);
}

void Controls::sendFeedback() {
    frc::SmartDashboard::PutString("Arm_currentmode", armMode ? "arm mode" : "hang mode");
    frc::SmartDashboard::PutNumber("Hang_Speed", MAX_ARM_SPEED);
    frc::SmartDashboard::PutNumber("Arm_Speed", armSpeed);

}