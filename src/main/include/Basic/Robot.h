// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <Controls/Controls.h>
#include <Drive/Drive.h>
#include <GamEpiece/Shamptake.h>
#include <Autonomous/Auto.h>
#include <GamEpiece/Arm.h>
#include <Hanger/Hang.h>

class Robot : public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

private:
    void reset(Mechanism::MatchMode mode);

    Mechanism::MatchMode lastMode = Mechanism::MatchMode::DISABLED;
    Drive drive;
    Shamptake shampTake;
    Arm arm;
    Hang hang;
    //Controls controls {nullptr, &shampTake, &arm, &hang};
    Controls controls {&drive, &shampTake, &arm, &hang};
    Auto autoCode {&drive, &shampTake, &arm};
    std::vector<Mechanism*> allMechanisms {
        &arm, &hang, &drive, &shampTake, &controls, &autoCode
    };
};