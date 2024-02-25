// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Basic/Robot.h>
#include <Autonomous/Auto.h>

void Robot::RobotInit() {
    AddPeriodic([&]() {
        for (Mechanism* mech : allMechanisms) {
            mech->sendFeedback();
        }
    }, 80_ms);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
    //reset(Mechanism::MatchMode::AUTO);
    //autoCode.doAuto();
    blinky.setLEDMode(BlinkyBlinky::LEDMode::OFF);
    reset(Mechanism::MatchMode::AUTO);
}
void Robot::AutonomousPeriodic() {
    //controls.process();
    autoCode.process();
    drive.process();
    arm.process();
    shampTake.process();
}

void Robot::TeleopInit() {
    blinky.setLEDMode(BlinkyBlinky::LEDMode::OFF);
    reset(Mechanism::MatchMode::TELEOP);
}
void Robot::TeleopPeriodic() {
    shampTake.process();
    controls.process();
    arm.process();
    //hang.process();
    shampTake.runIntakeMotors();

    drive.process();
    blinky.process();
}

void Robot::DisabledInit() {
    reset(Mechanism::MatchMode::DISABLED);
    blinky.setLEDMode(BlinkyBlinky::LEDMode::PIT_MODE);
}
void Robot::DisabledPeriodic() {
    controls.processInDisabled();
    blinky.process();    
    reset(Mechanism::MatchMode::DISABLED);
}
void Robot::DisabledPeriodic() {
    controls.processInDisabled();
}

void Robot::TestInit() {
    reset(Mechanism::MatchMode::TEST);
}
void Robot::TestPeriodic() {}

void Robot::reset(Mechanism::MatchMode mode) {
    for (Mechanism* mech : allMechanisms) {
       mech->callResetToMode(lastMode);
    }

    lastMode = mode;
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
