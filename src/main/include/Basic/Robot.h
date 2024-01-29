// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <Controls/Controls.h>
#include <Drive/Drive.h>
#include <Auto/Auto.h>

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
    Controls controls {&drive};
    Auto autoCode{&drive};
    std::vector<Mechanism*> allMechanisms {
        &drive, &controls
    };
};
