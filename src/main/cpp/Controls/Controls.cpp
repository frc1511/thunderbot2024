#include <Controls/Controls.h>
#include <cmath>
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>

#define AXIS_DEADZONE 0.1


Controls::Controls(Drive* _drive, Arm* _arm, Hang* _hang) :
    drive(_drive),
    //shamptake(nullptr),
    arm(_arm),
    hang(_hang),
    armMode(false) 
{

}

void Controls::resetToMode(MatchMode mode) { }

void Controls::process() {
    //driveController.process();
    auxController.process();
    doAux();


    // //doSwitchPanel();
    // if (callaDisable) {
    //     //drive->manualControlRelRotation(0, 0, 0, Drive::ControlFlag::BRICK);
    // }
    // else {
    //     //doDrive();
    // }

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
    //doSwitchPanel();

    using DriveButton = DriveControllerType::Button;

    bool resetOdometry = driveController.getButton(DriveButton::OPTIONS, ThunderGameController::ButtonState::PRESSED);
    bool calIMU = driveController.getButton(DriveButton::SHARE, ThunderGameController::ButtonState::PRESSED);

    if (resetOdometry) {
        //drive->resetOdometry();
    }

    if (calIMU) {
        //drive->calibrateIMU();
    }
}

bool Controls::getShouldPersistConfig() {
    //doSwitchPanel();

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

    if (auxController.getButton(AuxButton::TOUCH_PAD)){
        armMode = !armMode;
    }
    
    if (hangModeControls == true){
        /* hang and trap controls 


        */
       
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
    bool toggleCurve = auxController.getButton(AuxButton::B);
    bool shooter = auxController.getButton(AuxButton::LEFT_BUMPER);
    bool fire = auxController.getButton(AuxButton::X);
    bool intake = auxController.getButton(AuxButton::Y);
    bool outtake = auxController.getButton(AuxButton::A);
    // if (toggleCurve) {
    //     shamptake->shooterSwitch();
    //     printf("Shooter curved: %d\n", shamptake->shooterMode == shamptake->CURVED);
    // }

    // if (!intake) {
    //     shamptake->intakeSpeed = shamptake->STOP;
    // }

    // if (shooter) {
    //     if (fire){
    //         shamptake->intakeSpeed = shamptake->FIRE;
    //         shamptake->shooter(1);
    //         shamptake->trippedBefore = false;
    //         printf("RESET\n");
    //     } else {
    //         shamptake->shooter(0.6);
    //     }
    // } else {
    //     shamptake->shooter(0);
    // }
    
    // if (outtake) {
    //     shamptake->intakeSpeed = shamptake->OUTTAKE;
    //     shamptake->trippedBefore = false;
    //     printf("RESET\n");
    // }

     
     if (auxController.getDPad() == ThunderGameController::DPad::UP){
       //set arm low enough to get under the stage
        if(armMode) {
       arm->ARM_SLOW_SPEED += .1;
       if (arm->ARM_SLOW_SPEED >= .5) {
        arm->ARM_SLOW_SPEED = .5;
       }
        }
     } else if (auxController.getDPad() == ThunderGameController::DPad::DOWN){
    //     //set arm back to normal position
    if (armMode) {
    arm->ARM_SLOW_SPEED -= .1;
       if (arm->ARM_SLOW_SPEED <= -.5) {
        arm->ARM_SLOW_SPEED = -.5;
       }
    }
     }

    if (armMode){
        // Arm stuff- ALSO  A FUNCTIONNNN OUTTA DIS STUFF 2
        double armPivot = -auxController.getAxis(AuxAxis::LEFT_Y);

        if (std::fabs(armPivot) < AXIS_DEADZONE) {
            armPivot = 0;
        }

        if (armPivot > arm->ARM_SLOW_SPEED) {
            armPivot = arm->ARM_SLOW_SPEED;
        }
        
        if (armPivot < -arm->ARM_SLOW_SPEED) {
            armPivot = -arm->ARM_SLOW_SPEED;
        }
        currentSpeed = armPivot;
        frc::SmartDashboard::PutNumber("Arm_Speed", currentSpeed);
        if (arm != nullptr)
        {
            arm->setPower(armPivot);
        }

    }
    else{
        // Hang stuff - MAKE A FUNCTION OUTTA THIS STUFF
        double hangLeft = -auxController.getAxis(AuxAxis::LEFT_Y);
        double hangRight = -auxController.getAxis(AuxAxis::RIGHT_Y);

        if (std::fabs(hangLeft) < AXIS_DEADZONE) {
            hangLeft = 0;
        }

        if (hangLeft > MAX_ARM_SPEED) {
            hangLeft = MAX_ARM_SPEED;
        }
        if (hangLeft < -MAX_ARM_SPEED) {
            hangLeft = -MAX_ARM_SPEED;
        }
        if (hang != nullptr)
        {
            if (intake && outtake) {
                hang->setSolenoids(Hang::SolenoidStates::BOTH);
            } else if (intake) {
                hang->setSolenoids(Hang::SolenoidStates::LEFT);
            } else if (outtake) {
                hang->setSolenoids(Hang::SolenoidStates::RIGHT);
            } else {
                hang->setSolenoids(Hang::SolenoidStates::OFF);
            }
            //hang->setSpeed(hangLeft);
        }

        // Right Side

        if (std::fabs(hangRight) < AXIS_DEADZONE) {
            hangRight = 0;
        }
        if (hangRight > MAX_ARM_SPEED) {
            hangRight = MAX_ARM_SPEED;
        }
        if (hangRight < -MAX_ARM_SPEED) {
            hangRight = -MAX_ARM_SPEED;
        }
        if (hang != nullptr) {
            //hang->setSpeed(hangRight);
        }
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
    frc::SmartDashboard::PutString("Arm_currentmode", armMode?"arm test mode" : "hang mode");
    frc::SmartDashboard::PutNumber("Hang_Speed", MAX_ARM_SPEED);
    frc::SmartDashboard::PutNumber("Arm_Speed", currentSpeed);


}