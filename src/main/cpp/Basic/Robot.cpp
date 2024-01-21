// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Basic/Robot.h>

void Robot::RobotInit() {
    AddPeriodic([&]() {
        for (Mechanism* mech : allMechanisms) {
            mech->sendFeedback();
        }
    }, 40_ms);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
    reset(Mechanism::MatchMode::AUTO);
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    reset(Mechanism::MatchMode::TELEOP);
}
void Robot::TeleopPeriodic() {
    controls.process();
    drive.process();
}

void Robot::DisabledInit() {
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
